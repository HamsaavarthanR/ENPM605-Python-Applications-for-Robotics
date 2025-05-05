def compute_perimeter(a):
    """
    Compute the perimeter of a square.
        :math:`A = 4 \\times a`

    Args:
        a (float): The length of the side of the square.

    Returns:
        float: The perimeter of the square.
    """
    return 4 * a

def compute_area(a):
    """
    Compute the area of a square.
        :math:`A = a^2`

    Args:
        a (float): The length of the side of the square.

    Returns:
        float: The area of the square.
    """
    return a ** 2

if __name__ == "__main__":
    print("Testing the square module.")
    print(f"The perimeter of the square is {compute_perimeter(4)}.")
    print(f"The area of the square is {compute_area(4)}.")