import os
import pandas as pd

def convert_xlsx_to_csv(xlsx_filename, csv_filename):
    if not os.path.exists(xlsx_filename):
        print(f"Warning: File not found: {xlsx_filename} (Skipping)")
        return

    # Load Excel file
    df = pd.read_excel(xlsx_filename, header=None)

    if df.shape[0] < 2:  # Ensure at least 2 rows exist
        print(f"Error: {xlsx_filename} does not have at least 2 rows. Skipping.")
        return

    # Extract first and second rows
    first_row = df.iloc[0].dropna().astype(str).tolist()
    second_row = df.iloc[1].dropna().astype(str).tolist()

    # Determine the max length between the two rows
    num_entries = max(len(first_row), len(second_row))

    # Pad shorter row with empty strings
    first_row.extend([""] * (num_entries - len(first_row)))
    second_row.extend([""] * (num_entries - len(second_row)))

    # Create final CSV data
    csv_data = [["0", first, second, "0"] for first, second in zip(first_row, second_row)]

    # Save to CSV
    pd.DataFrame(csv_data).to_csv(csv_filename, index=False, header=False)
    print(f"Converted: {xlsx_filename} -> {csv_filename}")

if __name__ == "__main__":
    for i in range(1, 25):  # Process 1.xlsx to 24.xlsx
        xlsx_filename = f"{i}.xlsx"
        csv_filename = f"{i}.csv"
        convert_xlsx_to_csv(xlsx_filename, csv_filename)
