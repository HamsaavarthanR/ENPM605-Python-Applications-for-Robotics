def compute_perimeter(a, b, c):
    """
    Compute the perimeter of a triangle.
        :math:`perimeter = a + b + c`

    Args:
        a (float): The length of the side.
        b (float): The length of the second side.
        c (float): The length of the third side.

    Returns:
        float: The perimeter of the triangle.
    """
    return a + b + c


def compute_area(base, height):
    """
    Compute the area of a triangle.
        :math:`area = \\frac{1}{2} \\times base \\times height`

    Args:
        base (float): The length of the base of the triangle.
        height (float): The height of the triangle.

    Returns:
        float: The area of the triangle.
    """
    return 0.5 * base * height


# print("__name__:", __name__)

if __name__ == "__main__":
    print("Testing the triangle module.")
    print(f"The perimeter of the triangle is {compute_perimeter(3, 4, 5)}.")
    print(f"The area of the triangle is {compute_area(3, 4)}.")
