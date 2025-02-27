Аргументы cmd:
parser = argparse.ArgumentParser()
parser.add_argument("--src_dir", type=str, default="src_data", help="Исходная папка с данными")
parser.add_argument("--output_dir", type=str, default="augmented_data", help="Выходная папка")
parser.add_argument("--start_idx", type=int, default=0, help="Начальный индекс изображений")
parser.add_argument("--end_idx", type=int, default=None, help="Конечный индекс изображений")
args = parser.parse_args()


python run.py --src_dir src_data --output_dir augmented_data --start_idx 0 --end_idx 100