import os
from datetime import datetime

src_directory = "data/oneweb/"
dst_directory = "data/processed/"

target_date = datetime(2023,3,3)
target_epoch = int(target_date.strftime("%j"))

dst_filename = "oneweb_" + target_date.strftime("%Y-%m-%d") + ".tle"
dst_path = os.path.join(dst_directory, dst_filename)

with open(dst_path, "w") as dst_file:
    print(os.listdir(src_directory))
    for src_filename in os.listdir(src_directory):
        src_path = os.path.join(src_directory, src_filename)

        # loop through the file, choose rows with correct date
        with open(src_path, "r") as src_file:
            for sat_name in src_file:
                try:
                    tle_line1 = next(src_file)
                    tle_line2 = next(src_file)
                except StopIteration:
                    continue # missing TLE files with 2 rows

                if int(tle_line1[20:23]) == target_epoch:
                    print(sat_name)
                    dst_file.write(sat_name)
                    dst_file.write(tle_line1)
                    dst_file.write(tle_line2)


