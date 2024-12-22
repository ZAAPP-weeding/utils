import argparse
from pathlib import Path

import imageio
from moms_apriltag import ApriltagBoard  # lol


def make_board_with_ids(id_list):
    assert len(id_list) == 6
    return ApriltagBoard((2, 3), "tag25h9", 0.02, ids=id_list)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dest", type=str, required=True)
    args = parser.parse_args()

    dest_dir = Path(args.dest)

    # This requires updating the library! Otherwise you will generate two identical boards. Well, actually it'll crash.
    first_ids = list(range(6))
    second_ids = list(range(6, 12))

    board1 = make_board_with_ids(first_ids)
    print("board1.ids", board1.ids)
    board2 = make_board_with_ids(second_ids)
    print("board2.ids", board2.ids)

    assert set(board1.ids) & set(board2.ids) == set()

    for board, fn in zip(
        [board1, board2], ["apriltag_target_large1.png", "apriltag_target_large2.png"]
    ):
        tgt = board.board
        dest = dest_dir / fn
        imageio.imwrite(dest, tgt)
        print("Saved to", dest)


if __name__ == "__main__":
    main()
