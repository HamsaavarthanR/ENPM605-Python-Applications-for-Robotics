loyalty_customers = {"john doe", "jane smith", "alice brown"}


def check_loyalty(customer_name):
    """
    Check if a customer is a loyalty member.

    Args:
        customer_name (str): Name of a person

    Returns:
        bool: True if the customer is a loyalty customer, otherwise false
    """
    return customer_name.lower() in loyalty_customers
