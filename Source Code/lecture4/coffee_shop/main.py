import order_calculator
import customer_manager

def main():
    customer_name = input("Enter customer name: ")
    items_prices = [3.50, 2.75, 5.00]  # Example prices
    
    is_loyalty_member = customer_manager.check_loyalty(customer_name)
    
    total_cost = order_calculator.calculate_total(items_prices, is_loyalty_member)
    
    print(f"Customer: {customer_name}")
    print(f"Loyalty Member: {'Yes' if is_loyalty_member else 'No'}")
    print(f"Total Cost: ${sum(items_prices)}")
    print(f"Total Cost (discount): ${total_cost}")

if __name__ == "__main__":
    main()