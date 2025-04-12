import csv
import os

def write_to_csv(data):
  filename = "data.csv"
  file_exists = os.path.isfile(filename)

  # Field names (CSV headers)
  fieldnames = ["Iteration", "ZLB", "ZUB", "GAP", "Lambda", "theta_n", "fij", "yij", "s_lambda", "yij_1a", "fij_1a"]

  # Open file and write data
  with open(filename, mode='w', newline='') as file:
      writer = csv.DictWriter(file, fieldnames=fieldnames)

      # Write header if file is new
      writer.writeheader()

      writer.writerows(data)