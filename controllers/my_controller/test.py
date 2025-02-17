import math

def find_next_nearest(coords, current):
    """Finds the nearest coordinate to the given current coordinate."""
    min_distance = float('inf')
    nearest_coord = None

    for coord in coords:
        distance = math.sqrt((current[0] - coord[0]) ** 2 + (current[1] - coord[1]) ** 2)
        if distance < min_distance:
            min_distance = distance
            nearest_coord = coord

    return nearest_coord, min_distance

def rearangeGreenCoordinates(GREEN_CORDINATES, ENTRANCE):
    """Rearranges coordinates using a nearest-neighbor approach."""
    if not GREEN_CORDINATES:
        return []

    current = ENTRANCE
    sorted_coordinates = []
    remaining = GREEN_CORDINATES.copy()  # Copy to avoid modifying the original list

    while remaining:
        next_coord, _ = find_next_nearest(remaining, current)
        sorted_coordinates.append(next_coord)
        remaining.remove(next_coord)
        current = next_coord  # Move to the next coordinate

    return sorted_coordinates

# Example Usage:
ENTRANCE = [19, 10]
GREEN_CORDINATES = [[2, 3], [9, 6], [3, 7], [8, 6], [17, 0]]

GREEN_CORDINATES = rearangeGreenCoordinates(GREEN_CORDINATES, ENTRANCE)
print("Rearranged Coordinates:", GREEN_CORDINATES)
