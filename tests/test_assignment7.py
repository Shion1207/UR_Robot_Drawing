import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/assign7_bringup/scripts'))
try:
    from semantic_map import OccupancyMap, build_regions
    from voice_commands import CommandError, parse_command
except ImportError:
    OccupancyMap = build_regions = parse_command = None
    CommandError = ValueError

REGIONS = [
    {'id': 'A', 'name': 'Phòng học', 'aliases': ['a', 'khu vực a', 'khu a', 'phòng học']},
    {'id': 'B', 'name': 'Phòng thí nghiệm', 'aliases': ['b', 'bê', 'khu vực b', 'phòng thí nghiệm']},
    {'id': 'C', 'name': 'Hành lang', 'aliases': ['c', 'xê', 'khu vực c', 'hành lang', 'hành lang c']},
]


class CommandTests(unittest.TestCase):
    def setUp(self):
        self.assertIsNotNone(parse_command, 'Voice command parser not implemented')

    def test_single_destination_and_semantic_names(self):
        for text, expected in [('Đi đến khu vực A.', ['A']),
                               ('Đi đến phòng thí nghiệm', ['B']),
                               ('Di chuyển tới hành lang', ['C']),
                               ('Hãy đi đến phòng học nhé', ['A'])]:
            with self.subTest(text=text):
                self.assertEqual(parse_command(text, REGIONS), expected)

    def test_source_must_be_visited_before_destination(self):
        self.assertEqual(parse_command('Di chuyển từ khu vực A đến khu vực B', REGIONS), ['A', 'B'])

    def test_via_points_and_spoken_letter(self):
        self.assertEqual(parse_command('Đi từ phòng học qua hành lang C rồi đến bê', REGIONS), ['A', 'C', 'B'])

    def test_unknown_anywhere_rejects_entire_route(self):
        for text in ('Đi đến khu vực D', 'Đi từ khu vực D đến khu vực B',
                     'Đi từ A qua phòng họp đến B'):
            with self.subTest(text=text), self.assertRaises(CommandError):
                parse_command(text, REGIONS)

    def test_chatter_negation_and_missing_destination_do_not_move(self):
        for text in ('Tôi đang ở phòng học', 'Không đi đến A', 'Đừng đi đến A', 'Đi từ A', 'Đi đến'):
            with self.subTest(text=text), self.assertRaises(CommandError):
                parse_command(text, REGIONS)

    def test_stop_is_distinct_from_navigation(self):
        self.assertEqual(parse_command('Dừng lại', REGIONS), [])
        self.assertEqual(parse_command('Hủy điều hướng', REGIONS), [])


class MapTests(unittest.TestCase):
    def setUp(self):
        self.assertIsNotNone(OccupancyMap, 'Map segmentation not implemented')
        self.path = Path(__file__).resolve().parents[1] / 'src/assign7_bringup/maps/map.yaml'
        self.map = OccupancyMap(self.path)

    def test_pixel_world_round_trip_uses_cell_centres(self):
        for u, v in ((0, 0), (50, 60), (157, 117)):
            x, y = self.map.pixel_to_world(u, v)
            self.assertEqual(self.map.world_to_pixel(x, y), (u, v))
        x, y = self.map.pixel_to_world(0, 117)
        self.assertAlmostEqual(x, -3.925)
        self.assertAlmostEqual(y, -2.935)

    def test_map_origin_rotation_is_supported(self):
        self.map.origin = (2.0, 3.0, math.pi/2)
        x, y = self.map.pixel_to_world(0, 117)
        self.assertAlmostEqual(x, 1.975)
        self.assertAlmostEqual(y, 3.025)
        self.assertEqual(self.map.world_to_pixel(x, y), (0, 117))

    def test_regions_have_safe_distinct_goals_and_do_not_label_obstacles(self):
        seeds = [dict(REGIONS[0], seed=[-1.2, 1.4]), dict(REGIONS[1], seed=[2.7, 1.9]),
                 dict(REGIONS[2], seed=[0.0, -2.0])]
        result, labels = build_regions(self.map, seeds, clearance=0.4)
        self.assertEqual(len(result['regions']), 3)
        goals = []
        for i, r in enumerate(result['regions'], 1):
            u, v = self.map.world_to_pixel(*r['goal'][:2])
            self.assertGreaterEqual(float(self.map.clearance[v, u]), 0.4)
            self.assertEqual(int(labels[v, u]), i)
            goals.append((u, v))
        self.assertEqual(len(set(goals)), 3)
        self.assertTrue((labels[~self.map.free] == 0).all())

    def test_world_outside_map_is_rejected(self):
        with self.assertRaises(ValueError):
            self.map.world_to_pixel(100, 100)


if __name__ == '__main__':
    unittest.main()
