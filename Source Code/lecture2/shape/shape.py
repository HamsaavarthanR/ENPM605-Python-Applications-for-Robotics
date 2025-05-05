# import sys
# import os.path

# folder = (os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
# sys.path.append(folder)

# --- Approach #1 ---
import square.square  # noqa: E402
import triangle.triangle  # noqa: E402

square_perimeter = square.square.compute_perimeter(4)
print(f"The perimeter of the square is {square_perimeter}.")
square_area = square.square.compute_area(4)
print(f"The area of the square is {square_area}.")
triangle_perimeter = triangle.triangle.compute_perimeter(3, 4, 5)
print(f"The perimeter of the triangle is {triangle_perimeter}.")
triangle_area = triangle.triangle.compute_area(3, 4)
print(f"The area of the triangle is {triangle_area}.")

# # --- Approach #2 ---
# import square.square as square_mod
# import triangle.triangle as triangle_mod

# square_perimeter = square_mod.compute_perimeter(4)
# print(f"The perimeter of the square is {square_perimeter}.")
# square_area = square_mod.compute_area(4)
# print(f"The area of the square is {square_area}.")
# triangle_perimeter = triangle_mod.compute_perimeter(3, 4, 5)
# print(f"The perimeter of the triangle is {triangle_perimeter}.")
# triangle_area = triangle_mod.compute_area(3, 4)
# print(f"The area of the triangle is {triangle_area}.")

# # --- Approach #3 ---
# from square.square import (
#     compute_perimeter as square_compute_perimeter,
#     compute_area as square_compute_area
# )
# from triangle.triangle import (
#     compute_perimeter as triangle_compute_perimeter,
#     compute_area as triangle_compute_area
# )

# square_perimeter = square_compute_perimeter(4)
# print(f"The perimeter of the square is {square_perimeter}.")
# square_area = square_compute_area(4)
# print(f"The area of the square is {square_area}.")
# triangle_perimeter = triangle_compute_perimeter(3, 4, 5)
# print(f"The perimeter of the triangle is {triangle_perimeter}.")
# triangle_area = triangle_compute_area(3, 4)
# print(f"The area of the triangle is {triangle_area}.")

# # --- Approach #4 ---
# from square.square import *
# from triangle.triangle import *

# square_perimeter = compute_perimeter(4)
# print(f"The perimeter of the square is {square_perimeter}.")
# square_area = compute_area(4)
# print(f"The area of the square is {square_area}.")
# triangle_perimeter = compute_perimeter(3, 4, 5)
# print(f"The perimeter of the triangle is {triangle_perimeter}.")
# triangle_area = compute_area(3, 4)
# print(f"The area of the triangle is {triangle_area}.")


