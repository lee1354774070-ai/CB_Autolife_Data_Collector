"""Annotation contract tests; no hardware, ROS, or LeRobot installation needed."""

import json
import os
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

from subtask_annotations import SubtaskAnnotations, check_pending_annotations, parse_subtasks, prepare_annotation


class SubtaskTest(unittest.TestCase):
    def test_plan_validation(self):
        self.assertEqual(parse_subtasks('[]'), ())
        self.assertEqual(parse_subtasks('[" pick ", "pick"]'), ('pick', 'pick'))
        for value in ('', 'null', '{}', '"pick"', '[1]', '[""]', '["  "]', '[true]'):
            with self.subTest(value=value), self.assertRaises(ValueError):
                parse_subtasks(value)

    def test_boundaries_partition_confirmed_and_unknown_frames(self):
        annotations = SubtaskAnnotations(('pick', 'handover', 'place'))
        self.assertFalse(annotations.mark(30))
        self.assertFalse(annotations.mark(70))
        result = annotations.manifest(4, 'overall task', 90, 30, 'command:save')
        self.assertEqual(result['task'], 'overall task')
        self.assertEqual(result['segments'][1]['start_frame'], 30)
        self.assertEqual(result['segments'][1]['end_frame'], 70)
        self.assertEqual(result['unannotated_ranges'], [{'start_frame': 70, 'end_frame': 90}])
        self.assertTrue(result['needs_review'])
        self.assertEqual(annotations.progress()['next_subtask'], 'place')
        self.assertTrue(annotations.mark(90))
        complete = annotations.manifest(4, 'overall task', 90, 30, 'subtasks_complete')
        self.assertTrue(complete['annotation_complete'])
        self.assertEqual(complete['unannotated_ranges'], [])

    def test_no_marks_means_all_frames_unknown(self):
        result = SubtaskAnnotations(('pick',)).manifest(0, 'task', 10, 30, 'shutdown')
        self.assertEqual(result['segments'], [])
        self.assertTrue(result['needs_review'])
        self.assertEqual(result['unannotated_ranges'], [{'start_frame': 0, 'end_frame': 10}])

    def test_duplicate_empty_backwards_and_excess_marks_rejected(self):
        annotations = SubtaskAnnotations(('pick', 'place'))
        for count in (0, -1):
            with self.assertRaises(ValueError):
                annotations.mark(count)
        annotations.mark(2)
        for count in (1, 2):
            with self.assertRaises(ValueError):
                annotations.mark(count)
        self.assertEqual(annotations.ends, [2])
        annotations.mark(4)
        with self.assertRaises(ValueError):
            annotations.mark(5)
        with self.assertRaises(ValueError):
            annotations.manifest(0, 'task', 3, 30, 'save')

    def test_disabled_plan_and_reset(self):
        disabled = SubtaskAnnotations(())
        self.assertFalse(disabled.enabled)
        self.assertFalse(disabled.complete)
        with self.assertRaises(ValueError):
            disabled.mark(1)
        annotations = SubtaskAnnotations(('pick',))
        annotations.mark(3)
        annotations.reset()
        self.assertFalse(annotations.complete)
        self.assertEqual(annotations.progress()['confirmed'], 0)

    def test_writer_split_and_legacy_buffer_adapters(self):
        annotations = SubtaskAnnotations(('pick', 'place'))
        annotations.mark(2)
        for split in (False, True):
            owner = SimpleNamespace(episode_buffer={'size': 4, 'subtask_index': [-1] * 4})
            dataset = SimpleNamespace(writer=owner) if split else owner
            annotations.apply_to_buffer(dataset, 4)
            self.assertEqual([int(v[0]) for v in owner.episode_buffer['subtask_index']], [0, 0, -1, -1])

    def test_buffer_contract_mismatch_fails(self):
        for buffer in (None, {}, {'size': 2, 'subtask_index': []}, {'size': 3, 'subtask_index': [-1] * 3}):
            with self.assertRaises(RuntimeError):
                SubtaskAnnotations(('pick',)).apply_to_buffer(SimpleNamespace(episode_buffer=buffer), 2)

    def test_pending_marker_blocks_resume_and_existing_annotations_are_not_overwritten(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            annotations = SubtaskAnnotations(('pick',))
            manifest = annotations.manifest(1, 'overall', 4, 30, 'save')
            check_pending_annotations(root)
            pending, final = prepare_annotation(root, manifest)
            self.assertEqual(json.loads(pending.read_text()), manifest)
            with self.assertRaises(RuntimeError):
                check_pending_annotations(root)
            with self.assertRaises(FileExistsError):
                prepare_annotation(root, manifest)
            os.replace(pending, final)
            check_pending_annotations(root)
            with self.assertRaises(RuntimeError):
                prepare_annotation(root, manifest)


if __name__ == '__main__':
    unittest.main()
