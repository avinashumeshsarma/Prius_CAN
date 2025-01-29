import pandas as pd

# Load the CSV file (Change 'your_file.csv' to the actual file path)
df = pd.read_csv("/home/avinashumeshsarma/panda/examples/Data_Log/Jan 28/01.csv")

# Convert MessageID from Hexadecimal to Decimal
df["MessageID_Decimal"] = df["MessageID"].apply(lambda x: int(x, 16))

# Get Unique Message IDs
unique_message_ids = sorted(df["MessageID_Decimal"].unique())

# Display Unique Message IDs
print("Unique Message IDs (Decimal):")
print(unique_message_ids)