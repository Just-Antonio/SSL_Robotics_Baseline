import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'scripts')))

from color_sorter import color_compare, Color


def test_color_red():
    assert color_compare([255, 0, 0]) == Color.RED


def test_color_yellow():
    assert color_compare([255, 255, 0]) == Color.YELLOW


def test_color_blue():
    assert color_compare([0, 0, 255]) == Color.BLUE


def test_color_unknown():
    # a gray value should map to UNKNOWN
    assert color_compare([128, 128, 128]) == Color.UNKNOWN
