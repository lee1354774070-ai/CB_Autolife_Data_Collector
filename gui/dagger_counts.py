"""Read committed episode labels, never infer saves from button clicks."""
import json
from functools import lru_cache
from pathlib import Path


@lru_cache(maxsize=8)
def discarded_from_log(path, size, modified_ns):
    count = 0
    with Path(path).open() as stream:
        for line in stream:
            try:
                event = json.loads(line)
            except ValueError:
                continue  # A concurrently appended last line is not a receipt.
            count += event.get('event') == 'episode_discarded'
    return count


def collection_counts(base):
    base = Path(base)
    counts = dict(autonomous_success=0, autonomous_failure=0,
                  human_corrected_success=0, human_failure=0, unlabelled=0)
    info = base / 'dataset/meta/info.json'
    total = json.loads(info.read_text())['total_episodes'] if info.exists() else 0
    for index in range(total):
        path = base / 'dataset/annotations/dagger' / f'episode_{index:06d}.json'
        if not path.exists():
            counts['unlabelled'] += 1
            continue
        label = json.loads(path.read_text())
        if (label.get('episode_index') != index
                or type(label.get('task_success')) is not bool
                or type(label.get('human_takeover')) is not bool):
            raise ValueError(f'条目 {index} 标签不完整，计数未确认')
        success, human = label['task_success'], label['human_takeover']
        key = ('human_corrected_success' if human else 'autonomous_success') if success else (
            'human_failure' if human else 'autonomous_failure')
        counts[key] += 1
    status_path = base / '.official_recording_status.json'
    status = json.loads(status_path.read_text()) if status_path.exists() else {}
    log = base / 'dataset/sync_log.jsonl'
    if log.exists():
        stat = log.stat()
        counts['discarded'] = discarded_from_log(str(log), stat.st_size, stat.st_mtime_ns)
    else:
        counts['discarded'] = status.get('session_discarded_episodes', 0)
    counts['discard_confirmed'] = status.get('event') == 'discard' and status.get('success') is True
    return counts


def describe_counts(counts):
    text = (f"当前数据集：自主成功 {counts['autonomous_success']} · 自主失败 {counts['autonomous_failure']}"
            f" · 纠错成功 {counts['human_corrected_success']} · 接管后失败 {counts['human_failure']}"
            f"\n当前数据集已丢弃 {counts['discarded']} 条")
    if counts['unlabelled']:
        text += f" · 历史未标注 {counts['unlabelled']} 条"
    if counts['discard_confirmed']:
        text += ' · 本条丢弃已确认'
    return text
