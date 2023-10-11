import sqlite3
import csv
import pandas as pd 
import rclpy.serialization
from px4_msgs.msg import VehicleOdometry   
# Replace 'your_database.db3' with the name of your SQLite database file
db_file = 'px4_logger/bag_files/test_bag_1.db3'

# Replace 'output.csv' with the name you want for the CSV file
csv_file = 'output.csv'

# Connect to the SQLite database
conn = sqlite3.connect(db_file)

# Create a cursor object
cursor = conn.cursor()

# Execute an SQL query to select data from your database table (replace 'your_table' with the actual table name)
cursor.execute('SELECT * from messages')

# Fetch all the rows as a list of tuples
data = cursor.fetchall()

# Get the column names from the cursor description
columns = [desc[0] for desc in cursor.description]

# Write data to the CSV file
with open(csv_file, 'w', newline='') as csvfile:
    csv_writer = csv.writer(csvfile)
    
    # Write the column headers
    csv_writer.writerow(columns)
    
    # Write the data rows
    csv_writer.writerows(data)

# Close the database connection
conn.close()

data = pd.read_csv('output.csv')
serial_data = rclpy.serialization.deserialize_message((data['data'],'utf-8'),VehicleOdometry)
print(f'Data from {db_file} has been successfully exported to {csv_file}.')
