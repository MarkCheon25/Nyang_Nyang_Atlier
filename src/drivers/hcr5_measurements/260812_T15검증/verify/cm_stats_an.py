#!/usr/bin/env python3
"""V-6 재산출 — `/controller_manager/statistics/full` 스냅샷에서 주기를 뽑는다 (260812-감압밸브, 검B).

`pal_statistics_msgs/Statistics` 는 names 배열과 values 배열이 **같은 순서로 짝**을 이룬다.
`ros2 topic echo --once` 의 YAML 을 그대로 먹는다.

⚠️ periodicity 의 단위는 **Hz** 다 (µs 가 아니다) — 컨트롤러 2종이 100.01 로 나오는 것으로 확정됐다
   (260811-버스바). execution_time 은 µs.

⚠️ 이 스냅샷에는 missed call·timeout 이 없다. 그 수치는 `LoanedStateInterface` 가 **파괴될 때**
   (컨트롤러 비활성·프로세스 종료) 런치 로그에 WARN 으로 찍힌다 → `loaned_an.py` 가 담당.

사용: python3 cm_stats_an.py <cm_stats.yaml>
"""
import re
import sys
import pathlib

WANT = ("read_cycle/periodicity", "write_cycle/periodicity", "periodicity/average")


def load(path):
    """names: [...] / values: [...] 두 블록을 순서쌍으로 묶는다."""
    txt = path.read_text()
    names = re.findall(r"name:\s*(\S+)", txt)
    # values 블록은 name 과 짝을 이루는 value: 필드
    values = re.findall(r"value:\s*(\S+)", txt)
    if len(names) != len(values):
        print(f"⚠️ names {len(names)} ≠ values {len(values)} — 짝이 안 맞는다", file=sys.stderr)
    return dict(zip(names, values))


def main():
    p = pathlib.Path(sys.argv[1])
    d = load(p)
    print(f"=== {p.name} — 총 {len(d)} 항목 ===\n")

    print("--- read/write 주기 (단위 Hz) ---")
    for k in sorted(d):
        if "cycle/periodicity" in k and k.endswith(("average", "standard_deviation", "sample_count", "min", "max")):
            print(f"  {k:<62} {d[k]}")

    print("\n--- 컨트롤러 주기 (단위 Hz) ---")
    for k in sorted(d):
        if k.endswith("stats/periodicity/average") and "cycle" not in k:
            print(f"  {k:<62} {d[k]}")

    print("\n--- 실행시간 (단위 µs) ---")
    for k in sorted(d):
        if "execution_time/average" in k:
            print(f"  {k:<62} {d[k]}")


if __name__ == "__main__":
    main()
