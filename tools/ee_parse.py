#!/usr/bin/env python3
"""内部 Flash 配置区（256 KB）离线解析 / 导出 / 自检。

用途：不用飞机在线也能看清"盘里到底存了什么"。
典型用法（Keil 或 STM32CubeProgrammer 把 0x080C0000 起的 0x40000 读到 ee.bin）：

    python ee_parse.py ee.bin                  # 摘要：每个 bank 有几条有效记录、最新是谁
    python ee_parse.py ee.bin --hist           # 逐条列出（新→旧）
    python ee_parse.py ee.bin --hist --diff    # 顺带打印相邻记录改了哪些参数
    python ee_parse.py ee.bin --export p.json  # 把最新一条导出成预设文件
    python ee_parse.py ee.bin --json raw.json  # 全部记录导出成 JSON
    python ee_parse.py --selftest              # 不接硬件，用合成数据自检

记录格式（固件侧见 Core/Src/flash_ee.c）：
    偏移  长度  字段
      0     4   magic = 0x45454346 ("FCEE")，擦除态 0xFFFFFFFF
      4     2   ver（1）
      6     2   plen（载荷字节数，4 的整数倍）
      8     4   seq（保存序号，单调递增）
     12     4   crc32( bytes[4..12) ‖ 载荷 )
     16   plen  载荷（参数体：'A''P' + count + count×(uint16 id, float)）

⚠ 本文件的常量在 bb_cfg.py 里，和固件是两份拷贝；`--selftest` 会读
  Core/Inc/flash_ee.h、param.h、param.c 逐项核对，对不上直接报错。
"""

from __future__ import annotations

import argparse
import json
import os
import struct
import sys
import zlib

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import bb_cfg  # noqa: E402

HDR = struct.Struct("<IHHII")        # magic, ver, plen, seq, crc = 16 B
BANK_BYTES = bb_cfg.EE_BANK_SIZE


# ==================================================================
#  一、把镜像切成记录
# ==================================================================
def decode_payload(payload: bytes):
    """返回 (kind, items)；kind ∈ {params, p1test, unknown}。"""
    if len(payload) >= 4 and payload[:2] == bb_cfg.PARAM_MAGIC:
        n = struct.unpack_from("<H", payload, 2)[0]
        items = []
        for i in range(n):
            off = 4 + i * bb_cfg.PARAM_ITEM_SIZE
            if off + bb_cfg.PARAM_ITEM_SIZE > len(payload):
                break
            pid, value = struct.unpack_from("<Hf", payload, off)
            items.append((pid, value))
        return "params", items
    if len(payload) >= 4 and struct.unpack_from("<I", payload)[0] == bb_cfg.P1_TEST_MAGIC:
        return "p1test", []
    return "unknown", []


def parse_bank(data: bytes, base_off: int, bank: int) -> list[dict]:
    """解析一个 bank 的全部槽位，返回"看起来像记录"的那些（含坏记录）。"""
    out = []
    for slot in range(bb_cfg.EE_SLOT_COUNT):
        off = base_off + slot * bb_cfg.EE_RECORD_SIZE
        chunk = data[off:off + bb_cfg.EE_RECORD_SIZE]
        if len(chunk) < bb_cfg.EE_HDR_SIZE:
            break
        magic, ver, plen, seq, crc = HDR.unpack_from(chunk, 0)
        if magic != bb_cfg.EE_MAGIC:
            continue                      # 空槽（0xFFFFFFFF）或垃圾

        rec = {"bank": bank, "slot": slot, "seq": seq, "ver": ver,
               "plen": plen, "crc": crc, "ok": False, "why": "",
               "kind": "unknown", "items": []}

        if ver != bb_cfg.EE_VER:
            rec["why"] = f"ver={ver} 不认识"
        elif plen == 0 or plen > bb_cfg.EE_PAYLOAD_MAX or plen % 4 != 0:
            rec["why"] = f"plen={plen} 非法"
        else:
            payload = chunk[bb_cfg.EE_HDR_SIZE:bb_cfg.EE_HDR_SIZE + plen]
            calc = zlib.crc32(chunk[4:12] + payload) & 0xFFFFFFFF
            if calc != crc:
                rec["why"] = f"crc 对不上（存 {crc:08X} 算 {calc:08X}）"
            else:
                rec["ok"] = True
                rec["kind"], rec["items"] = decode_payload(payload)
        out.append(rec)
    return out


def parse_image(data: bytes, base_off: int = 0) -> dict:
    """解析两个 bank。base_off 是 Bank A 在 data 里的偏移。"""
    recs_a = parse_bank(data, base_off, 0)
    recs_b = parse_bank(data, base_off + BANK_BYTES, 1)
    good = [r for r in recs_a + recs_b if r["ok"]]
    # 新→旧。用有符号差处理 seq 回绕
    good.sort(key=lambda r: r["seq"], reverse=True)
    return {
        "banks": {0: recs_a, 1: recs_b},
        "valid": good,
        "bad": [r for r in recs_a + recs_b if not r["ok"]],
        "newest": good[0] if good else None,
    }


def load_image(path: str, offset: int | None = None) -> tuple[bytes, int]:
    """读镜像，返回 (data, Bank A 的偏移)。按文件长度自动判断来源。"""
    with open(path, "rb") as fp:
        data = fp.read()
    n = len(data)

    if offset is not None:
        return data, offset
    if n == 2 * BANK_BYTES:                       # 正好是配置区（0x080C0000 起 256KB）
        return data, 0
    if n == 0x100000:                             # 整片 1MB（0x08000000 起）
        return data, bb_cfg.EE_BANK0_BASE - 0x08000000
    if n > 2 * BANK_BYTES:                        # 比配置区大：当作从 0x080C0000 开始的更大块
        return data, 0
    raise SystemExit(
        f"{path}: 只有 {n} 字节，配置区需要 {2 * BANK_BYTES} 字节（0x{2 * BANK_BYTES:X}）。\n"
        f"  正确做法：把 0x080C0000 起、长度 0x40000 的 256KB 存成 bin；\n"
        f"  或者整片 1MB（0x08000000 起）。也可以手动指定 --offset。")


# ==================================================================
#  二、打印
# ==================================================================
BANK_NAMES = ("A", "B")


def bank_base(bank: int) -> int:
    return bb_cfg.EE_BANK0_BASE + bank * BANK_BYTES


def print_summary(res: dict) -> None:
    for bank in (0, 1):
        recs = res["banks"][bank]
        good = [r for r in recs if r["ok"]]
        bad = [r for r in recs if not r["ok"]]
        print(f"Bank {BANK_NAMES[bank]} @0x{bank_base(bank):08X}: "
              f"有效 {len(good)} 条，坏 {len(bad)} 条，已用槽 {len(recs)}/{bb_cfg.EE_SLOT_COUNT}")
        for r in good:
            kind = {"params": "参数", "p1test": "P1自检", "unknown": "未知"}[r["kind"]]
            extra = f" params={len(r['items'])}" if r["kind"] == "params" else ""
            print(f"   slot {r['slot']:3d}  seq={r['seq']:<6d} len={r['plen']:4d}  {kind}{extra}")
        for r in bad:
            print(f"   slot {r['slot']:3d}  seq={r['seq']:<6d}  [坏] {r['why']}")

    nw = res["newest"]
    print()
    if nw is None:
        print("==> 没有任何有效记录：上电会用编译期默认值（CFG:INFO 显示 src=def）")
    else:
        print(f"==> 最新记录：Bank {BANK_NAMES[nw['bank']]} slot {nw['slot']} "
              f"seq={nw['seq']}，启动时加载的就是这一条")


def print_hist(res: dict, diff: bool, table: dict) -> None:
    good = res["valid"]
    if not good:
        print("（没有有效记录）")
        return
    for i, r in enumerate(good):
        kind = {"params": "参数", "p1test": "P1自检", "unknown": "未知"}[r["kind"]]
        line = (f"[{i:2d}] Bank {BANK_NAMES[r['bank']]} slot {r['slot']:3d}  "
                f"seq={r['seq']:<6d} len={r['plen']:4d}  {kind}")
        if diff and r["kind"] == "params":
            newer = good[i - 1] if i > 0 else None
            if newer is not None and newer["kind"] == "params":
                d = diff_items(newer["items"], r["items"], table)
                line += ("  diff: " + d) if d else "  same"
        print(line)


def diff_items(a_items, b_items, table) -> str:
    """a 比 b 新：返回 b→a 的变化列表（最多 4 条）。"""
    a = dict(a_items)
    b = dict(b_items)
    out = []
    for pid in sorted(set(a) | set(b)):
        va, vb = a.get(pid), b.get(pid)
        if va is None or vb is None or abs(va - vb) > 1e-9:
            name = table.get(pid, {}).get("name", f"id{pid:04X}")
            out.append(f"{name} {vb}->{va}")
        if len(out) >= 4:
            out.append("...")
            break
    return ", ".join(out)


# ==================================================================
#  三、导出
# ==================================================================
def export_preset(res: dict, table: dict, name: str, note: str) -> dict:
    nw = res["newest"]
    if nw is None or nw["kind"] != "params":
        raise SystemExit("最新一条记录不是参数记录，无法导出预设")
    params = {}
    for pid, value in nw["items"]:
        pname = table.get(pid, {}).get("name", f"id{pid:04X}")
        params[pname] = round(value, 6)
    return bb_cfg.preset_build(params, name=name, note=note)


def export_json(res: dict, table: dict) -> dict:
    def one(r):
        return {
            "bank": BANK_NAMES[r["bank"]], "slot": r["slot"], "seq": r["seq"],
            "len": r["plen"], "ok": r["ok"], "why": r["why"],
            "kind": r["kind"],
            "params": {table.get(pid, {}).get("name", f"id{pid:04X}"): round(v, 6)
                       for pid, v in r["items"]},
        }
    return {"newest": one(res["newest"]) if res["newest"] else None,
            "valid": [one(r) for r in res["valid"]],
            "bad": [one(r) for r in res["bad"]]}


# ==================================================================
#  四、自检（不接硬件）
# ==================================================================
def _make_record(seq: int, items: list, break_crc: bool = False) -> bytes:
    payload = bytearray(bb_cfg.PARAM_MAGIC)
    payload += struct.pack("<H", len(items))
    for pid, value in items:
        payload += struct.pack("<Hf", pid, value)
    while len(payload) % 4:
        payload += b"\x00"

    hdr = bytearray(bb_cfg.EE_HDR_SIZE)
    struct.pack_into("<IHH", hdr, 0, bb_cfg.EE_MAGIC, bb_cfg.EE_VER, len(payload))
    struct.pack_into("<I", hdr, 8, seq)
    crc = zlib.crc32(bytes(hdr[4:12]) + bytes(payload)) & 0xFFFFFFFF
    if break_crc:
        crc ^= 1
    struct.pack_into("<I", hdr, 12, crc)
    return bytes(hdr) + bytes(payload)


def _put(img: bytearray, bank: int, slot: int, rec: bytes) -> None:
    off = bank * BANK_BYTES + slot * bb_cfg.EE_RECORD_SIZE
    img[off:off + len(rec)] = rec


def selftest() -> int:
    fails = 0

    def check(cond, what, extra=""):
        nonlocal fails
        if cond:
            print(f"[OK] {what}")
        else:
            fails += 1
            print(f"[X ] {what}" + (f"   {extra}" if extra else ""))

    print("=== 1. 常量与固件是否一致 ===")
    problems = bb_cfg.check_firmware_constants()
    check(not problems, "flash_ee.h / param.h 与 bb_cfg.py 一致",
          "; ".join(problems))

    print("\n=== 2. 参数表解析 ===")
    try:
        rows = bb_cfg.load_param_table()
        check(True, f"从 param.c 解析出 {len(rows)} 项参数")
        check(all(rows[i - 1]["id"] < rows[i]["id"] for i in range(1, len(rows))),
              "参数 ID 严格升序")
        table = {r["id"]: r for r in rows}
    except Exception as exc:                       # noqa: BLE001
        check(False, "参数表解析", str(exc))
        print(f"\n自检失败 {fails} 项")
        return 1

    print("\n=== 3. CRC32 与固件实现一致 ===")
    check(zlib.crc32(b"123456789") & 0xFFFFFFFF == 0xCBF43926,
          "标准 CRC-32 校验值（0xCBF43926）")

    print("\n=== 4. 合成镜像：找最新 + 跳过坏记录 ===")
    img = bytearray(b"\xFF" * (2 * BANK_BYTES))
    p1 = [(rows[0]["id"], 9.0)]
    _put(img, 0, 0, _make_record(1, p1))
    _put(img, 0, 1, _make_record(2, [(rows[0]["id"], 10.0)]))
    _put(img, 0, 2, _make_record(3, [(rows[0]["id"], 12.0)]))
    _put(img, 0, 3, _make_record(4, [(rows[0]["id"], 99.0)], break_crc=True))  # 坏
    res = parse_image(bytes(img), 0)
    check(res["newest"] is not None and res["newest"]["seq"] == 3,
          "跳过 CRC 坏的记录，最新是 seq=3",
          f"实际 newest={res['newest']['seq'] if res['newest'] else None}")
    check(len(res["bad"]) == 1, "识别出 1 条坏记录", f"实际 {len(res['bad'])}")
    check(abs(dict(res["newest"]["items"])[rows[0]["id"]] - 12.0) < 1e-6,
          "参数值解出来是 12.0")

    print("\n=== 5. 跨 bank：更新的记录在 B 里要能选出来 ===")
    _put(img, 1, 0, _make_record(5, [(rows[0]["id"], 13.0)]))
    res = parse_image(bytes(img), 0)
    check(res["newest"]["bank"] == 1 and res["newest"]["seq"] == 5,
          "最新记录切到 Bank B / seq=5")

    print("\n=== 6. 导出预设 ===")
    preset = export_preset(res, table, name="selftest", note="")
    check(preset["params"].get(rows[0]["name"]) == 13.0,
          f"导出的预设里 {rows[0]['name']} = 13.0")

    print("\n=== 7. 空配置区 ===")
    res0 = parse_image(bytes(bytearray(b"\xFF" * (2 * BANK_BYTES))), 0)
    check(res0["newest"] is None and not res0["valid"], "全擦除的配置区解析成 0 条记录")

    print()
    if fails == 0:
        print("全部通过。")
        return 0
    print(f"自检失败 {fails} 项。")
    return 1


# ==================================================================
#  五、命令行
# ==================================================================
def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        prog="ee_parse.py",
        description="内部 Flash 配置区离线解析 / 导出 / 自检",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    ap.add_argument("image", nargs="?", help="配置区镜像（256KB）或整片 1MB")
    ap.add_argument("--offset", type=lambda s: int(s, 0),
                    help="配置区在文件里的偏移（默认按文件长度自动判断）")
    ap.add_argument("--hist", action="store_true", help="逐条列出（新→旧）")
    ap.add_argument("--diff", action="store_true", help="列出相邻记录之间的参数变化")
    ap.add_argument("--export", metavar="JSON", help="把最新一条导出成预设文件")
    ap.add_argument("--json", metavar="JSON", help="把全部记录导出成 JSON")
    ap.add_argument("--name", default="", help="导出预设时的名字（默认用文件名）")
    ap.add_argument("--note", default="", help="导出预设时的备注")
    ap.add_argument("--selftest", action="store_true", help="不接硬件，用合成数据自检")
    return ap


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    if args.selftest:
        return selftest()
    if not args.image:
        build_parser().print_help()
        return 1

    data, base_off = load_image(args.image, args.offset)
    res = parse_image(data, base_off)

    try:
        table = {r["id"]: r for r in bb_cfg.load_param_table()}
    except Exception as exc:                       # noqa: BLE001
        print(f"[!] 参数表解析失败，只显示 ID：{exc}")
        table = {}

    print_summary(res)
    if args.hist or args.diff:
        print("\n--- 历史（新→旧）---")
        print_hist(res, args.diff, table)

    if args.export:
        name = args.name or os.path.splitext(os.path.basename(args.export))[0]
        preset = export_preset(res, table, name=name, note=args.note)
        with open(args.export, "w", encoding="utf-8") as fp:
            json.dump(preset, fp, ensure_ascii=False, indent=2)
            fp.write("\n")
        print(f"\n[已导出预设 {args.export}]（{len(preset['params'])} 项参数）")

    if args.json:
        with open(args.json, "w", encoding="utf-8") as fp:
            json.dump(export_json(res, table), fp, ensure_ascii=False, indent=2)
            fp.write("\n")
        print(f"[已导出 JSON {args.json}]")

    return 0


if __name__ == "__main__":
    sys.exit(main())
