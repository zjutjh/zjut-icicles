import csv
from collections import defaultdict
from pathlib import Path

try:
    import matplotlib.pyplot as plt
except ImportError as exc:
    raise SystemExit(
        "matplotlib is required to generate plots. Install it with: pip install matplotlib"
    ) from exc

BASE_DIR = Path(__file__).resolve().parent
PLOTS_DIR = BASE_DIR / "plots"
PLOTS_DIR.mkdir(exist_ok=True)


def read_csv(path):
    with path.open("r", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def to_float(value):
    return float(value)


def to_int(value):
    return int(value)


def save_line_plot(x, series, title, xlabel, ylabel, output_name):
    plt.figure(figsize=(9, 5.5))
    for label, y in series:
        plt.plot(x, y, marker="o", linewidth=2, label=label)
    plt.title(title)
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.grid(True, linestyle="--", alpha=0.4)
    if len(series) > 1:
        plt.legend()
    plt.tight_layout()
    plt.savefig(PLOTS_DIR / output_name, dpi=200)
    plt.close()


def main():
    windows_rows = read_csv(BASE_DIR / "windows_summary.csv")
    linux_rows = read_csv(BASE_DIR / "linux_summary.csv")

    windows_threads = [to_int(row["thread_count"]) for row in windows_rows]
    windows_times = [to_float(row["avg_time_seconds"]) for row in windows_rows]
    windows_speedups = [to_float(row["speedup"]) for row in windows_rows]
    windows_efficiencies = [to_float(row["efficiency"]) for row in windows_rows]

    save_line_plot(
        windows_threads,
        [("Windows laptop", windows_times)],
        "Windows laptop: thread count vs runtime",
        "Thread count",
        "Average runtime (s)",
        "windows_runtime.png",
    )

    save_line_plot(
        windows_threads,
        [("Windows laptop", windows_speedups)],
        "Windows laptop: thread count vs speedup",
        "Thread count",
        "Speedup",
        "windows_speedup.png",
    )

    save_line_plot(
        windows_threads,
        [("Windows laptop", windows_efficiencies)],
        "Windows laptop: thread count vs efficiency",
        "Thread count",
        "Parallel efficiency",
        "windows_efficiency.png",
    )

    linux_by_size = defaultdict(list)
    for row in linux_rows:
        linux_by_size[to_int(row["matrix_size"])].append(row)

    ordered_sizes = sorted(linux_by_size)
    linux_runtime_series = []
    linux_speedup_series = []
    linux_efficiency_series = []
    x_threads = None

    for size in ordered_sizes:
        rows = linux_by_size[size]
        rows.sort(key=lambda row: to_int(row["thread_count"]))
        threads = [to_int(row["thread_count"]) for row in rows]
        times = [to_float(row["avg_time_seconds"]) for row in rows]
        speedups = [to_float(row["speedup"]) for row in rows]
        efficiencies = [to_float(row["efficiency"]) for row in rows]

        x_threads = threads
        linux_runtime_series.append((f"N={size}", times))
        linux_speedup_series.append((f"N={size}", speedups))
        linux_efficiency_series.append((f"N={size}", efficiencies))

    save_line_plot(
        x_threads,
        linux_runtime_series,
        "Linux server: thread count vs runtime",
        "Thread count",
        "Average runtime (s)",
        "linux_runtime.png",
    )

    save_line_plot(
        x_threads,
        linux_speedup_series,
        "Linux server: thread count vs speedup",
        "Thread count",
        "Speedup",
        "linux_speedup.png",
    )

    save_line_plot(
        x_threads,
        linux_efficiency_series,
        "Linux server: thread count vs efficiency",
        "Thread count",
        "Parallel efficiency",
        "linux_efficiency.png",
    )

    print(f"Plots generated in: {PLOTS_DIR}")


if __name__ == "__main__":
    main()
