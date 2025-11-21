import argparse
import os
import csv
from typing import Iterable, Mapping, Sequence

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


LOG_COLUMNS = [
    "Time_ms",
    "com_Pos_deg",
    "pul_Pos_deg",
    "com_Vel_deg_per_s",
    "pul_Vel_deg_per_s",
]


def save_csv(data_log: Iterable[Sequence[float]], filename: str = "scurve_run.csv") -> str:
    os.makedirs("logs", exist_ok=True)
    filepath = os.path.join("logs", filename)
    with open(filepath, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(LOG_COLUMNS)
        writer.writerows(data_log)
    print(f"-- 실행 완료! CSV 저장: {filepath} --")
    return filepath


def moving_average(data: np.ndarray, window: int = 10) -> np.ndarray:
    """이동 평균 필터(유효 구간)."""
    if data.size == 0 or window <= 1 or data.size < window:
        return data.copy()
    kernel = np.ones(window, dtype=float) / window
    return np.convolve(data, kernel, mode="valid")


def as_dataframe(data_log: Iterable[Sequence[float]] | Mapping[str, Sequence[float]]) -> pd.DataFrame:
    """데이터 로그 입력을 DataFrame 형태로 정규화."""
    if isinstance(data_log, Mapping):
        return pd.DataFrame(data_log)
    return pd.DataFrame(data_log, columns=LOG_COLUMNS)


def cumulative_abs(series: np.ndarray) -> np.ndarray:
    """증가 누적치를 위해 절대 증분을 합산."""
    if series.size == 0:
        return series.copy()
    diffs = np.diff(series, prepend=series[0])
    cum = np.cumsum(np.abs(diffs))
    return cum


def plot_results(data_log: Iterable[Sequence[float]] | Mapping[str, Sequence[float]] | str, ma_window: int = 10):
    """CSV 경로나 로그 데이터를 받아 속도/위치 그래프를 생성."""
    if isinstance(data_log, str):
        df = pd.read_csv(data_log)
    else:
        df = as_dataframe(data_log)

    # 필요한 컬럼이 모두 있는지 확인
    missing = [col for col in LOG_COLUMNS if col not in df.columns]
    if missing:
        raise ValueError(f"다음 컬럼이 누락되었습니다: {missing}")

    time = df["Time_ms"].to_numpy(dtype=float) * 1e-3
    com_vel = df["com_Vel_deg_per_s"].to_numpy(dtype=float)
    pul_vel = df["pul_Vel_deg_per_s"].to_numpy(dtype=float)
    com_pos_raw = df["com_Pos_deg"].to_numpy(dtype=float)
    pul_pos_raw = df["pul_Pos_deg"].to_numpy(dtype=float)

    # 위치는 절대 증분을 누적하여 (항상 증가) 표시
    com_pos = cumulative_abs(com_pos_raw)
    pul_pos = cumulative_abs(pul_pos_raw)

    # 속도 이동 평균
    pul_vel_filtered = moving_average(pul_vel, window=ma_window)
    time_filtered = time[: len(pul_vel_filtered)]

    plt.figure(figsize=(8, 6))

    # ---------------- Velocity ----------------
    plt.subplot(2, 1, 1)
    plt.plot(time, com_vel, "--", label="Commanded Vel [deg/s]")
    if pul_vel.size:
        plt.plot(time, pul_vel, ":", label="PUL-based Vel raw [deg/s]", alpha=0.6)
    if pul_vel_filtered.size:
        plt.plot(time_filtered, pul_vel_filtered, label=f"PUL-based Vel [deg/s] (MA{ma_window})", alpha=0.8)
    plt.ylabel("Angular Velocity [deg/s]")
    plt.legend()
    plt.grid(True)

    # ---------------- Position ----------------
    plt.subplot(2, 1, 2)
    plt.plot(time, com_pos, label="Commanded Pos cumulative [deg]")
    plt.plot(time, pul_pos, label="PUL-based Pos cumulative [deg]", alpha=0.8)
    plt.xlabel("Time [s]")
    plt.ylabel("Angle [deg]")
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.show()


def main():
    parser = argparse.ArgumentParser(description="Exported 모션 CSV를 시각화합니다.")
    parser.add_argument("csv_path", help="분석할 CSV 경로")
    parser.add_argument("--ma-window", type=int, default=10, help="이동 평균 윈도 크기 (samples)")
    args = parser.parse_args()

    plot_results(args.csv_path, ma_window=args.ma_window)


if __name__ == "__main__":
    main()
