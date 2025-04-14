##----------------------------------------------------------------------------------------------------------##
### RWA1 - INVENTORY MANAGEMENT SYSTEM ###
##----------------------------------------------------------------------------------------------------------##

import datetime
# from pprint import pprint

class InventoryManagementSystem():

    def __init__(self):

        # Create a dictionary to store all the details about a list of products
        self.products_dict = dict()
        # Variable to store last modified timestamp
        self.timestamp = None
        # Variables to store category counts
        self.electronics_count = 0 # Quantity of Electronics available
        self.books_count = 0 # Quantity of Books available
        self.food_count = 0 # Quantity of Food available

        # Print Introductory message
        print("===== Inventory Management System =====", '\n')
        print("Current Categories: Electronics, Books, Food", '\n')
        

    ## ADD PRODUCT (a) ##
    def add_product(self, name, price, quantity, category, suppliers):
        # Add the product 'name' to the 'products_dict' and set it's value as another dictionary of details
        self.products_dict[name] = {'price':price, 
                                    'quantity':quantity, 
                                    'category': category,
                                    'suppliers':suppliers}

        # Update the category counts
        if category == 'Electronics':
            self.electronics_count += quantity
        elif category == 'Books':
            self.books_count += quantity
        else:
            self.food_count += quantity

        # Last Modified Timestamp
        self.timestamp = datetime.datetime.now()
        print("Product added successfully! Last modified: ", self.timestamp, '\n')


    ## REMOVE PRODUCT (r) ##
    def remove_product(self, name):
        # Variables to store product 'category' and 'quantity
        category = self.products_dict[name]['category']
        quantity = self.products_dict[name]['quantity']

        # If availability of product is zero or less
        if quantity <= 0:
            print("No more items available to remove for the given product!")
        # If the quantity of product available is > 0
        else:            
            # Remove Product from the dictionary
            self.products_dict.pop(name)

            # Update the category counts
            if category == 'Electronics':
                self.electronics_count -= quantity
            elif category == 'Books':
                self.books_count -= quantity
            else:
                self.food_count -= quantity
            
            # Last Modified Timestamp
            self.timestamp = datetime.datetime.now()
            print("Product removed successfully! Last modified: ", self.timestamp, '\n')


    ## SEARCH PRODUCTS (s) ##
    def search_product(self, *args):
        choice = args[0]

        # Create a dictionary to store details of products found
        products_found = dict()
        # Variable to store number of results found
        num = 0

        # If choice == 1 (Search by: Price range)
        if choice == 1:
            # Obtain 'min' and 'max' range of prices
            min = args[1]
            max = args[2]
            for name in self.products_dict:
                # Obtain 'price' of each product
                price = self.products_dict[name]['price']
                if  (price >= min) and (price <= max):
                    # Obtain 'category' and 'quantity' of products_found
                    category = self.products_dict[name]['category']
                    quantity = self.products_dict[name]['quantity']
                    # Store the details in 'products_found'
                    products_found[name] = {'price':price, 'category':category, 'quantity':quantity}
                    num += 1


        # If choice == 2 (Search by: Category)
        elif choice == 2:
            category_input = args[1] # Input categoty
            for name in self.products_dict:
                # Obtain 'category' of each product
                category = self.products_dict[name]['category']
                if  category == category_input:
                    # Obtain 'price' and 'quantity' of products_found
                    price = self.products_dict[name]['price']
                    quantity = self.products_dict[name]['quantity']
                    # Store the details in 'products_found'
                    products_found[name] = {'price':price, 'category':category, 'quantity':quantity}
                    num += 1
                

        # If choice == 3 (Search by: Supplier)
        elif choice == 3:
            supplier_input = args[1] # Input Supplier
            for name in self.products_dict:
                # Obtain 'suppliers' of each product
                suppliers = self.products_dict[name]['suppliers']
                if  supplier_input in suppliers:
                    # Obtain 'price', 'category' and 'quantity' of products_found
                    price = self.products_dict[name]['price']
                    category = self.products_dict[name]['category']
                    quantity = self.products_dict[name]['quantity']
                    # Store the details in 'products_found'
                    products_found[name] = {'price':price, 'category':category, 'quantity':quantity}
                    num += 1

        # Print results: products_found
        print(f"Results: ({num} result(s) found)!")
        if num > 0:
            for product in products_found:
                price_found = products_found[product]['price']
                category_found = products_found[product]['category']
                quantity_found = products_found[product]['quantity']
                print(f" - {product} (${price_found}) - {category_found} - {quantity_found} units")


    ## EDIT PRODUCT (e) ##
    def edit_product(self, name):
        # Obtain details of the product from the products_dict
        print("Current details from dictionary: ")
        print(f" - Price: ${self.products_dict[name]['price']}")
        print(f" - Quantity: {self.products_dict[name]['quantity']}")
        print(f" - Category: {self.products_dict[name]['category']}")
        print(f" - Suppliers: {self.products_dict[name]['suppliers']}")

        # Obtain edit option from the user
        print("What would you like to edit?")
        print("1. Price \n2. Quantity \n3. Suppliers \n4. Multiple fields")
        while True: 
            try:
                choice = int(input("Choice: "))
                # Check if valid choice was made, if not repeat
                if choice not in [1, 2, 3, 4]:
                    print("Error: Enter valid options from above!")
                else:
                    break
            except ValueError:
                print("Error: Choice can only be an integer option (1, 2, 3, 4)!")

        # If choice == 1 (Edit Price)
        if choice == 1:
            while True:
                try:
                    new_price = float(input("New price: "))
                    # Check if valid price was given, if not repeat
                    if new_price < 0:
                        print("Error: Price cannot be negative!")
                    else:
                        break
                except ValueError:
                    print("Error: Price can only be a positive number!")
            # Update 'price'
            self.products_dict[name]['price'] = new_price

        # If choice == 2 (Edit Quantity)
        elif choice == 2:
            while True:
                try:
                    new_quantity = int(input("New quantity: "))
                    # Check if valid quantity was given, if not repeat
                    if new_quantity < 0:
                        print("Error: Quantity cannot be negative!")
                    else:
                        break
                except ValueError:
                    print("Error: Quantity can only be a positive integer!")
            # Update 'quantity'
            self.products_dict[name]['quantity'] = new_quantity

        # If choice == 3 (Edit Suppliers)
        elif choice == 3:
            while True:
                new_suppliers = [str(supplier) for supplier in input("New suppliers (comma-seperated): ").split(', ')]
                # Check if valid quantity was given, if not repeat
                if new_suppliers == ['']:
                    print("Error: Suppliers cannot be empty!")
                else:
                    break
            # Update 'quantity'
            self.products_dict[name]['suppliers'] = new_suppliers

        # If choice == 4 (Edit Multiple fields)
        elif choice == 4:
            # Edit all options
            #(Edit Price)
            while True:
                try:
                    new_price = float(input("New price: "))
                    # Check if valid price was given, if not repeat
                    if new_price < 0:
                        print("Error: Price cannot be negative!")
                    else:
                        break
                except ValueError:
                    print("Error: Price can only be a positive number!")
            # Update 'price'
            self.products_dict[name]['price'] = new_price

            #(Edit Quantity)
            while True:
                try:
                    new_quantity = int(input("New quantity: "))
                    # Check if valid quantity was given, if not repeat
                    if new_quantity < 0:
                        print("Error: Quantity cannot be negative!")
                    else:
                        break
                except ValueError:
                    print("Error: Quantity can only be a positive number!")
            # Update 'quantity'
            self.products_dict[name]['quantity'] = new_quantity

            #(Edit Suppliers)
            while True:
                new_suppliers = [str(supplier) for supplier in input("New suppliers (comma-seperated): ").split(', ')]
                # Check if valid quantity was given, if not repeat
                if new_suppliers == ['']:
                    print("Error: Suppliers cannot be empty!")
                else:
                    break
            # Update 'quantity'
            self.products_dict[name]['suppliers'] = new_suppliers
        
        self.timestamp = datetime.datetime.now()
        print("Updated successfully! Last modified: ", self.timestamp, '\n')


    ## REPORT PRODUCTS (t) ##
    def report(self):
        print("===== Inverntory Report =====")
        
        # Initate variable to store total inventory value
        total = 0.0
        # Initate a dict for low stock alert
        low_stock = dict()

        for name in self.products_dict:
            # Obtain product value = quantity*price
            product_quantity = self.products_dict[name]['quantity']
            product_price = self.products_dict[name]['price']
            product_value = product_quantity*product_price
            total += product_value

            # update low_stock dictionary
            if product_quantity < 5:
                low_stock[name] = product_quantity

        # Print total inventory value
        print("Total Inventory Value: $", total)
        print("Low Stock Alert (< 5 units): ")
        if len(low_stock) == 0:
            print("None")
        else:
            for items in low_stock:
                print(f" - {items}: {low_stock[items]} units")
        # Print items per category
        print("\nItems per category: ")
        print(f"Electronics: {self.electronics_count}\nBooks: {self.books_count}\nFood: {self.food_count}\n")
        # Products available
        print("Products available: ")
        if len(self.products_dict) == 0:
            print("None")
        else:
            for product in self.products_dict.keys():
                print(f" - {product} (${self.products_dict[product]['price']}) - {self.products_dict[product]['quantity']} units")

    
    ## GET PRODUCTS (g) ##
    def get_products(self):
        return self.products_dict

## ========================================================================================================= ##

# Create a dictionary of list of products to start an Inventory Management System (IMS) 
ims = InventoryManagementSystem()
ims.add_product('Smart Watch', 199.99, 25, 'Electronics', ['Samsung', 'BestTech', 'TechWorld'])
ims.add_product('Origin', 100.99, 30, 'Books', ['IntPublishers', 'LocalPulblishers', 'Oxford'])
ims.add_product('Cereals', 49.99, 60, 'Food', ['AgroFoods', 'CerealMarket', 'AGSources'])
ims.add_product('Laptop', 1200.00, 20, 'Electronics', ['MacBook', 'Predator', 'ASUS'])
ims.add_product('Fundamental Physics', 149.99, 40, 'Books', ['Walnut Publications', 'Wiley', 'Stanford'])

print("Initial dictionary created for Inventory: ")

## ========================================================================================================= ##

## Entering User Input Mode ##
print("============== Entering User Input Mode ==================", '\n')
print("Current Categories: Electronics, Books, Food", '\n')
print("(**NOTE: User inputs are case-sensitive!!)\n")

while True:
    print("=============================================")
    while True:
        user_input = input("Enter operation (a/r/e/s/t/q): ")
        if user_input not in ['a', 'r', 'e', 's', 't', 'q']:
            print("Error: Enter valid operation!")
        else:
            break
    print("=============================================")
    # ADD PRODUCT
    if user_input == 'a':
        print("Adding new product: ")
        # Get 'name'
        while True:
            name_input = input("Name: ")
            if len(name_input) == 0:
                print("Error: Name cannot be an empty input!")
            else: 
                break
        # Get 'price'
        while True:
            # Use try-except to validate user input
            try:
                price_input = float(input("Price: "))
                if price_input <= 0:
                    print("Error: Price cannot be zero or negative!")
                else:
                    break
            except ValueError:
                print("Error: Price can only be a positive number!")
        # Get 'quantity'
        while True:
            # Use try-except to validate user input
            try:
                quantity_input = int(input("Quantity: "))
                if quantity_input <= 0:
                    print("Error: Quantity cannot be zero or negative!")
                else:
                    break
            except ValueError:
                print("Error: Quantity can only be a positive integer!")
        # Get 'category'
        while True:
            category_input = input("Category: ")
            if category_input not in ['Electronics', 'Books', 'Food']:
                print("Error: Enter valid category ('Electronics', 'Books', 'Food'). NOTE: User-inputs are case-sensitive!")
            else:
                break
        # Get 'suppliers'
        while True:
            suppliers_input = [str(supplier) for supplier in input("Suppliers (comma-seperated): ").split(', ')]
            if suppliers_input == ['']:
                print("Error: Supplier input cannot be empty!")
            else:
                break
        
        # Call Add function to add product to the dictionary
        ims.add_product(name_input, price_input, quantity_input, category_input, suppliers_input)

    # REMOVE PRODUCT
    elif user_input == 'r':
        print("Removing a product: ")
        while True:
            rem_name = input("Enter product name: ")
            if rem_name in ims.get_products().keys():
                break
            else:
                print("Error: Product name not found in dictionary. NOTE: User inputs are case-sensitive!")
        # Call remove function
        ims.remove_product(rem_name)
    
    # EDIT PRODUCT
    elif user_input == 'e':
        while True:
            product_name = input("Enter product name: ")
            if product_name in ims.get_products().keys():
                break
            else:
                print("Error: Product name not found in dictionary. NOTE: User inputs are case-sensitive!")
        # Call edit function
        ims.edit_product(product_name)

    # SEARCH PRODUCT
    elif user_input == 's':
        print("Search by: ")
        print("1. Price range \n2. Category \n3. Supplier")
        # Obtain valid user choice
        while True:
            try:
                user_choice = int(input("Enter choice (1-3): "))
                if user_choice not in [1, 2, 3]:
                    print("Error: Enter valid search option!")
                else:
                    break
            except ValueError:
                print("Error: Search option can only be an integer (1, 2, 3)!")
        
        # If user_choice == 1 (Price range)
        if user_choice == 1:
            while True:
                try:
                    min = float(input("Enter minimum price: "))
                    max = float(input("Enter maximum price: "))
                    if max <= min:
                        print("Error: max. limit cannot be equal or less than min. limit!")
                    if min < 0 or max < 0:
                        print("Error: Price range cannot be negative!")
                    else: 
                        break
                except ValueError:
                    print("Error: Price range can only be positive numbers!")
            # Call search function
            ims.search_product(1, min, max)

        # If user_choice == 2 (Category)
        elif user_choice == 2:
            while True:
                cat = input("Enter category: ")
                if cat not in ['Electronics', 'Books', 'Food']:
                    print(f"Error: {cat} category not found. Enter valid category ('Electronics', 'Books', 'Food')!")
                else:
                    break
            # Call search function
            ims.search_product(2, cat)

        # If user_choice == 3 (Supplier)
        elif user_choice == 3:
            while True: 
                sup = input("Enter supplier: ")
                if sup == [''] or len(sup) == 0:
                    print("Error: Supplier cannot be empty!")
                else:
                    break
            # Call search function
            ims.search_product(3, sup)

    # REPORT PRODUCT
    elif user_input == 't':
        ims.report()

    # QUIT PROGRAM
    elif user_input == 'q':
        break


print("Exiting program. Goodbye!")
