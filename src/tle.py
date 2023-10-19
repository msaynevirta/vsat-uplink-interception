import os
from datetime import datetime
from itertools import islice

src_directory = "data/oneweb/"
dst_directory = "data/processed/"

target_date = datetime(2023,2,28)
target_epoch = int(target_date.strftime("%j"))

dst_filename = "oneweb_" + target_date.strftime("%Y-%m-%d") + ".tle"
dst_path = os.path.join(src_directory, dst_filename)

with open(dst_path, "w") as dst_file:
    for src_filename in os.listdir(src_directory):
        src_path = os.path.join(src_directory, src_filename)

        # loop through the file, choose rows with correct date
        with open(src_path, "r") as src_file:
            lines_gen = islice(src_file, 3)
            for sat_name, tle_line1, tle_line2 in grouped(src_file, 3):
                src_epoch = int(tle_line1[20:23])
                if src_epoch == target_epoch:
                    #dst_file.write(sat_name)
                    #dst_file.write(tle_line1)
                    #dst_file.write(tle_line2)

                    print(sat_name, end="")          
