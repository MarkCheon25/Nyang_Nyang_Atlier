#!/usr/bin/env python3
"""V-6 재산출 — LoanedStateInterface 손실 통계를 런치 로그에서 뽑는다 (260812-감압밸브, 검B).

기준(확정 문면, 세션분할계획 §3):
  **기능 무해성** — CM 통계 missed call **<5%** · timeout **<0.1%**.
  값 갱신 4계층은 기록으로만 (원래 문면 "값 갱신 30±3Hz" 는 로봇이 정지하면 원리적으로 측정 불가).

⚠️ 이 통계는 `/controller_manager/statistics/full` 에 **없다.** `LoanedStateInterface` 가
   **파괴될 때**(컨트롤러 비활성·프로세스 종료) 런치 로그에 WARN 으로 한 번 찍힌다.
   → V-9(두절)나 정상 종료를 태워야 나온다. 읽기 창을 닫는 순서가 곧 이 행의 측정 순서다.

사용: python3 loaned_an.py <B_launch.log>
"""
import re
import sys
import pathlib

PAT = re.compile(
    r"LoanedStateInterface (\S+) has (\d+) \(([\d.]+) %\) timeouts "
    r"and (\d+) \(([\d.]+) %\) missed calls out of (\d+) get_value calls"
)
MISSED_MAX = 5.0    # %
TIMEOUT_MAX = 0.1   # %


def main():
    log = pathlib.Path(sys.argv[1])
    seen, rows = set(), []
    for m in PAT.finditer(log.read_text(errors="replace")):
        name, n_to, p_to, n_mi, p_mi, calls = m.groups()
        # 같은 인터페이스가 두 번 찍히는 경우가 있다(비활성 → 종료).
        # 호출 수가 가장 많은 판(=가장 긴 창)만 남긴다.
        rows.append((name, int(n_to), float(p_to), int(n_mi), float(p_mi), int(calls)))

    best = {}
    for r in rows:
        if r[0] not in best or r[5] > best[r[0]][5]:
            best[r[0]] = r

    print(f"=== LoanedStateInterface 손실 — 인터페이스 {len(best)}종 ===")
    print(f"{'인터페이스':<20}{'timeout':>12}{'missed':>14}{'호출':>10}")
    for name in sorted(best):
        _, n_to, p_to, n_mi, p_mi, calls = best[name]
        print(f"  {name:<18}{n_to:>5} ({p_to:6.4f}%){n_mi:>6} ({p_mi:6.4f}%){calls:>10}")

    worst_mi = max(best.values(), key=lambda r: r[4])
    worst_to = max(best.values(), key=lambda r: r[2])
    print(f"\n최악 missed  : {worst_mi[0]} {worst_mi[4]:.4f}%  [기준 <{MISSED_MAX}%]  "
          f"{'통과' if worst_mi[4] < MISSED_MAX else '실패'}")
    print(f"최악 timeout : {worst_to[0]} {worst_to[2]:.4f}%  [기준 <{TIMEOUT_MAX}%]  "
          f"{'통과' if worst_to[2] < TIMEOUT_MAX else '실패'}")


if __name__ == "__main__":
    main()
