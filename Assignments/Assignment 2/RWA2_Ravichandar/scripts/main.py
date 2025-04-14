## RWA_2 MAIN ##

# Import custom python module
import sensor_pipeline

# -----------------------------------------------------------------------------------#

## Building the Sensor Data Processing Pipeline


def process_sensor_pipeline(filename):
    """
    Function that integrates all the functions defined in 'sensor_pipeline' into a data processing pipepline.
    
    This function potentially utilises the functions from 'sensor_pipeline' module to retrieve data from a CSV File.
    Processes sensor data to convert 'distance' values (in inches) to centimeters. Filters valid sensor readings
    by applying specific threshold (20 cm) to the 'distance' readings. Prints out well-structured of the valid sensor readings.
    Finally, computes and prints the average distance obtained from all the valid sensor readings (that met the threshold).

    Args:
        filename (str): File path to the sensor_data.csv file.
    """

    # Read the sensor data from the CSV file.
    sensor_data = sensor_pipeline.read_sensor_data(filename=filename)

    # Process each sensor reading by converting the distance (in inches) to centimeters,
    # Using map() method from 'functools' module imported in sensor_pipeline
    sensor_data_processed = []

    for data in sensor_data:
        processed_data = sensor_pipeline.process_reading(data)
        sensor_data_processed.append(processed_data)

    # Filter the processed readings with specialized threshold filter.
    # Store the 'valid_distance_readings' and 'valid_sensor_readings' in seperate lists.
    valid_distance_readings, valid_sensor_readings = sensor_pipeline.filter_above_20(
        reading=sensor_data_processed
    )

    # Include error handling if no valid readings are avaiable
    valid_list_size = len(valid_distance_readings)
    if valid_list_size != 0:
        # Print each valid sensor reading.
        print(
            "\n||----- Valid Sensor Readings with 'distance' Above (or) Equal to 20 cm -----||\n"
        )
        print(
            f"Total Sensors that Meet Threshold ('distance' >= 20cm): {valid_list_size}\n"
        )
        for i, readings in enumerate(valid_sensor_readings):
            print(
                f"{i + 1}. Sensor: {readings['sensor_id']}, Distance: {readings['distance']}cm, Timestamp: {readings['timestamp']}"
            )
        print(
            "\n||---------------------------------------------------------------------------||\n"
        )

        # Compute and print the average distance of valid readings.
        valid_distance_avg = sensor_pipeline.average_distance(valid_distance_readings)

        print(
            f"||--------Average Distance of {valid_list_size} Valid Sensor Readings (shown above)--------||\n"
        )
        print(
            f"                     Average Valid Distance = {round(valid_distance_avg, 4)}cm                \n"
        )
        print(
            "||---------------------------------------------------------------------------||"
        )
    else:
        print("No Valid Sensor Readings are Avaiable for the given Threshold!")


# -----------------------------------------------------------------------------------#

if __name__ == "__main__":
    """
    _summary_
    """
    # Obtain file path
    file_path = "/home/infinity/enpm605/RWA2_Ravichandar/config/sensor_data.csv"

    # Call data processing pipeline
    process_sensor_pipeline(filename=file_path)

# -----------------------------------------------------------------------------------#
