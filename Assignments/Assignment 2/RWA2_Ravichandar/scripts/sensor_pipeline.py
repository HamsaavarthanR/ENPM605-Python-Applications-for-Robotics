### RWA 2 ###

# Import necessary modules
import csv
from functools import partial
from functools import reduce

# -----------------------------------------------------------------------------------#

## Reading and Parsing CSV Data ##


def read_sensor_data(filename):
    """
    Reads comma-seperated data from a CSV file and returns a list of dictionaries.

    Args:
        filename (str): File path to the CSV file.

    Returns:
        list: A list of dictionaries, where each dictionary represents a row in the CSV file.
              Keys of the dictionaries are the column headers from the first row of the CSV.
              Returns an empty list if the file is empty or an error occurs.
    """

    # Initiate an empty list to store the data from CSV file
    data = []

    # Use try and except to validate the 'filename' entered by the user.
    try:
        # Use 'open' method open a .csv file in "r" (read-only) mode.
        with open(filename, "r") as file:
            # Use 'DictReader' method from the 'csv' module to real each row of the .csv file and
            # store it in a dictionary who's keys will be their respective first row elements.
            csv_dict_reader = csv.DictReader(file)

            # Obtain each dictionary and store in the 'data' list.
            for row in csv_dict_reader:
                # Convert 'sensor_id' values to int.
                row["sensor_id"] = int(row["sensor_id"])
                # Convert 'distance' values to float.
                row["distance"] = float(row["distance"])

                data.append(row)

    # Exception: 'FileNotFoundError'
    except FileNotFoundError:
        print(f"Error: File not found at {filename}")
    # Exception: Others
    except Exception as e:
        print(f"An error occurred while reading CSV file: {e}")

    # Return the list of data dictionaries.
    return data

# -----------------------------------------------------------------------------------#

## Unit Conversion Function ##

def inches_to_cm(inches: float):
    """
    Pure function to convert distance measurements from 'inches' to centimeters 'cm' (with no side-effects).

    Args:
        inches (float): Distance measurement (in inches).

    Returns:
        float: Distance values of the input measurement converted to 'cm' (in centimeters).
    """
    
    # 1 inch = 2.54 cm
    return inches*2.54

# -----------------------------------------------------------------------------------#

## Processing Sensor Readings ##

def process_reading(reading: dict):
    """
    Function to process each sensor reading (dictionary) to apply unit conversion on the 'distance' measurement.

    Args:
        reading (dict): Individual sensor reading (which is a dictionary with exactly three keys.)

    Returns:
        dict: A copy of the original dictionary, after performing unit conversion on the distance measurement.
              The 'distance' measurement (in inches) of the given sensor dictionary is converted in to centimeters.
              Later, stored in a copy to return. Returns None is an error occured.
    """
    try:
        # Create a copy of the original dictionary 
        # (shallow copy is enough since the 'sensor' dictionaries are not nested)
        reading_copy = dict(reading)
        
        # Check if it's a valid sensor dictionary with exactly three keys.
        keys_to_check = ('sensor_id', 'distance', 'timestamp')
        if all(key in reading_copy for key in keys_to_check):
            # Convert 'distance' value from inches to centimeters.
            reading_copy['distance'] = round(inches_to_cm(reading_copy['distance']), 3)
            
            # Return the updated reading
            return reading_copy
        else:
            print(f"Error: One or more keys from {keys_to_check} is not found in '{reading}' dictionary.")
    
    # Exception: 'TypeError'
    except TypeError:
        print(f"Error: Invalid input '{reading}'. Argument must be a dictionary.")
    # Exception: Others
    except Exception as e:
        print(f"An error occured: {e}")
        
# -----------------------------------------------------------------------------------#
        
## Filtering Sensor Readings Based on a Threshold ##

def threshold_filter(threshold: float, reading: list):
    """
    Filters out the sensor readings' 'distance' measurement (in centimeters) that meets or exceeds the specific safety threshold.

    Args:
        threshold (float): Threshold value to verify safety conditions.
        reading (list): List of dictionaries (with exactly three keys) that stores all the sensor readings.

    Returns:
        list: List that stores all the 'distance' values that has met the given safety threshold.
              Returns an empty list if none of the sensor readings has met the safety condition or an error occurs.
        list: List of dictionaries all valid sensor readings that has met the given safety threshold.
              Returns an empty list if none of the sensor readings has met the safety condition or an error occurs.
    """
    
    #Create an empty list to store the sensor readings and distance that meet the given threshold
    valid_sensor = []
    valid_distance = []
    
    # Validate and verify individual sensor reading
    for sensor_reading in reading:
        try:
            # Create a copy of the original dictionary
            # (shallow copy is enough since the 'sensor_reading' dictionary is not nested)
            reading_copy = dict(sensor_reading)

            # Check if it's a valid sensor dictionary with exactly three keys.
            keys_to_check = ("sensor_id", "distance", "timestamp")
            if all(key in reading_copy for key in keys_to_check):
                # Validate appropriate 'threshold' input.
                try:
                    # Verify whether the reading's 'ditance' (in centimeters) meets or exceeds the given 'threshold'
                    if reading_copy["distance"] >= threshold:
                        # Add 'distance' reading to the 'valid_distance' list
                        valid_distance.append(reading_copy["distance"])
                        # Store sensor readings for later print usage.
                        valid_sensor.append(reading_copy)
                # Exception: 'TypeError'
                except TypeError:
                    print(f"Error: Invalid input '{threshold}'.  Argument must be int (or) float.")
                # Exception: Others
                except Exception as err:
                    print(f"An error occured while comparing reading's 'distance' and 'threshold': {err}")
                    
            else:
                print(
                    f"Error: One or more keys from {keys_to_check} is not found in '{reading}' dictionary.")

        # Exception: 'TypeError'
        except TypeError:
            print(f"Error: Invalid input '{reading}'. Argument must be a dictionary.")
        # Exception: Others
        except Exception as e:
            print(f"An error occured while processing '{reading}': {e}")
            
    # Return 'valid_distance' list of readings that met the given threshold
    return valid_distance, valid_sensor

# -----------------------------------------------------------------------------------#

## Filtering Sensor Readings above 20 cms ##

# Use partial() method from 'functools' module to create a specialized filter,
# that fixes the threshold to 20 cm.
filter_above_20 = partial(threshold_filter, threshold=20)

# -----------------------------------------------------------------------------------#

## Computing the Average Distance ##

def average_distance(readings: list):
    """
    Function to calculate the average distance (in centimeters) from the list of filtered sensor readings.

    Args:
        readings (list): List of sensor readings 'distance' (in centimeters)

    Returns:
        float: Returns the average of the list of sensor readings given, i.e., distances (in centimeters)
    """
    try:
        # Initiate a variable 'avg' to store the average distance of the 'readings' given
        avg = None
        # Obtain the length of the 'readings' list
        list_size = len(readings)
        
        if list_size != 0:
            list_sum = reduce(lambda x, y: x+y, readings)
            # Calculate avg = list_sum/list_size
            avg = list_sum/list_size
            
        else:
            print(f"Error: {readings} can not be an empty list")
    
    # Exception: 'TypeError'
    except TypeError:
        print(f"Error: Invalid input {readings}. Argument must be a non-empty list.") 
    # Exception: Others
    except Exception as e:
        print(f"An error occured while calculating {readings} average: {e}")
        
    # Return 'avg' 
    return avg

# -----------------------------------------------------------------------------------#
        