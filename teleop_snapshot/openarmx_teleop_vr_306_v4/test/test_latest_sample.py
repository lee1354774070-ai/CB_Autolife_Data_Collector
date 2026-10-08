from openarmx_teleop_vr_306_v4.latest_sample import LatestSampleMailbox


def test_pending_sample_is_replaced_and_old_sequence_is_invalidated():
    mailbox = LatestSampleMailbox()
    first = mailbox.put('first')
    second = mailbox.put('second')
    assert second == first + 1
    assert mailbox.take() == (second, 'second')
    assert not mailbox.is_latest(first)
    assert mailbox.is_latest(second)
    assert mailbox.stats()['replaced_pending'] == 1


def test_inflight_work_becomes_stale_when_new_sample_arrives():
    mailbox = LatestSampleMailbox()
    first = mailbox.put('first')
    assert mailbox.take() == (first, 'first')
    second = mailbox.put('second')
    assert not mailbox.is_latest(first)
    assert mailbox.is_latest(second)


def test_close_unblocks_empty_mailbox():
    mailbox = LatestSampleMailbox()
    mailbox.close()
    assert mailbox.take() is None
    assert mailbox.put('late') is None


def test_reset_discards_pending_and_invalidates_inflight_work():
    mailbox = LatestSampleMailbox()
    inflight = mailbox.put('inflight')
    assert mailbox.take() == (inflight, 'inflight')
    pending = mailbox.put('pending')
    reset_sequence = mailbox.reset()
    assert reset_sequence > pending
    assert not mailbox.is_latest(inflight)
    assert not mailbox.is_latest(pending)
    assert mailbox.stats()['pending'] is False
