"""Optional real CUDA decode tests use local synthetic JPEGs, never a robot."""
import os
import unittest
from unittest.mock import Mock, patch

import cv2
import numpy as np

from jpeg_decoding import NvJpegDecoder, select_jpeg_decoder


class DecoderSelectionTest(unittest.TestCase):
    def test_opencv_does_not_initialize_cuda(self):
        with patch('jpeg_decoding.NvJpegDecoder') as factory:
            self.assertIsNone(select_jpeg_decoder('opencv', 'cuda:0', Mock()))
        factory.assert_not_called()

    def test_auto_falls_back_only_at_initialization(self):
        logger = Mock()
        with patch('jpeg_decoding.NvJpegDecoder', side_effect=ImportError('no CUDA codec')):
            self.assertIsNone(select_jpeg_decoder('auto', 'cuda:0', logger))
            logger.warning.assert_called_once()
            with self.assertRaisesRegex(RuntimeError, 'initialization failed'):
                select_jpeg_decoder('nvjpeg', 'cuda:0', logger)
        decoder = Mock()
        with patch('jpeg_decoding.NvJpegDecoder', return_value=decoder):
            self.assertIs(select_jpeg_decoder('auto', 'cuda:0', logger), decoder)
        decoder.decode.side_effect = RuntimeError('CUDA failure')
        with self.assertRaisesRegex(RuntimeError, 'CUDA failure'):
            decoder.decode([(b'jpeg', 1, 1)])


@unittest.skipUnless(os.environ.get('COLLECTOR_TEST_NVJPEG') == '1', 'set COLLECTOR_TEST_NVJPEG=1 for real CUDA decode')
class NvJpegTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.decoder = NvJpegDecoder()

    def jpeg(self, color, size=(80, 64), progressive=False):
        width, height = size
        image = np.empty((height, width, 3), np.uint8)
        image[:] = color  # OpenCV input BGR.
        ok, data = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 95,
                                             cv2.IMWRITE_JPEG_PROGRESSIVE, int(progressive)])
        self.assertTrue(ok)
        return data.tobytes(), width, height

    def test_batch_order_rgb_shape_and_owned_output(self):
        import torch
        self.assertEqual(self.decoder.device, torch.device('cuda:0'))
        entries = [self.jpeg((20, 60, 230)), self.jpeg((210, 150, 40), (96, 48))]
        images = self.decoder.decode(entries)
        for entry, output in zip(entries, images, strict=True):
            reference = cv2.imdecode(np.frombuffer(entry[0], np.uint8), cv2.IMREAD_COLOR)[..., ::-1]
            self.assertEqual(output.shape, reference.shape)
            self.assertEqual(output.dtype, np.uint8)
            self.assertTrue(output.flags.c_contiguous)
            self.assertLessEqual(int(np.abs(output.astype(int) - reference).max()), 2)
        first = images[0].copy()
        for _ in range(50):
            self.decoder.decode(list(reversed(entries)))
        np.testing.assert_array_equal(images[0], first)
        self.assertFalse(np.shares_memory(images[0], images[1]))

    def test_progressive_and_padded_shm_slots(self):
        jpeg, width, height = self.jpeg((80, 100, 160), progressive=True)
        normal, padded = self.decoder.decode([(jpeg, width, height), (jpeg + bytes(4096), width, height)])
        np.testing.assert_array_equal(normal, padded)

    def test_invalid_jpeg_and_wrong_dimensions_are_not_reused(self):
        good = self.jpeg((50, 70, 90))
        for invalid in [(b'not a jpeg', 80, 64), (good[0], 160, 64)]:
            with self.subTest(invalid_size=len(invalid[0])), self.assertRaises((RuntimeError, ValueError)):
                self.decoder.decode([good, invalid])
        self.assertEqual(len(self.decoder.decode([good])), 1)
        self.assertEqual(self.decoder.decode([]), [])

    def test_unavailable_device_and_wrong_device_output_fail(self):
        import torch
        with self.assertRaisesRegex(ValueError, 'not visible'):
            NvJpegDecoder('cuda:999999')
        with patch.object(self.decoder, '_decode_jpeg', return_value=[torch.zeros(3, 64, 80, dtype=torch.uint8)]):
            with self.assertRaisesRegex(RuntimeError, 'expected cuda:0'):
                self.decoder.decode([self.jpeg((50, 70, 90))])


if __name__ == '__main__':
    unittest.main()
