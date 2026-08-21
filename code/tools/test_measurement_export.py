import csv
import sys
from pathlib import Path

ROOT = Path(__file__).parent
sys.path.insert(0, str(ROOT))

from measurement_export import normalize_rows, read_csv_rows, write_normalized_csv


def test_sd_csv_centimetres_are_normalized_with_explicit_time(tmp_path):
    source = tmp_path / "sd.csv"
    source.write_text(
        "timestamp_ns,receive_timestamp_ns,x,y,z,unit,validity,confidence\n"
        "100,110,100,200,50,cm,VALID,0.8\n",
        encoding="utf-8")
    rows = normalize_rows(
        read_csv_rows(source), target_id="target_0", frame_id="uwb_raw",
        source_mode="SOURCE_DIRECT_UWB")
    assert rows[0]["x_m"] == 1.0
    assert rows[0]["y_m"] == 2.0
    assert rows[0]["unit"] == "m"
    output = tmp_path / "normalized.csv"
    write_normalized_csv(output, rows)
    with output.open(encoding="utf-8", newline="") as stream:
        written = list(csv.DictReader(stream))
    assert written[0]["frame_id"] == "uwb_raw"


def test_invalid_rows_get_an_explicit_reason():
    rows = normalize_rows([{
        "timestamp_ns": "100", "x_m": "1", "y_m": "2", "z_m": "3",
        "validity": "INVALID",
    }])
    assert rows[0]["invalid_reason"] == "source_marked_invalid"
