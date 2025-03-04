from geopy.distance import geodesic
from shapely.geometry import Polygon, Point


class Sector:
    """
    Класс для представления сектора круга.

    Параметры:
    - center: tuple (longitude, latitude) — координаты центра сектора.
    - azimuth: float — азимут центральной оси сектора (в градусах).
    - angle: float — угол сектора (в градусах).
    - radius: float — длина радиуса сектора (в метрах).
    """

    def __init__(self, center, azimuth, angle, radius):
        self.center = center
        self.azimuth = azimuth
        self.angle = angle
        self.radius = radius

        # Создаём полигон
        self.polygon = Polygon(self._generate_points())

    def _generate_points(self):
        """
        Генерирует точки для построения сектора.
        """
        lon_center, lat_center = self.center

        # Вычисляем начальный и конечный азимуты сектора
        start_azimuth = self.azimuth - self.angle / 2
        end_azimuth = self.azimuth + self.angle / 2

        # Генерируем точки по окружности
        num_points = self.angle * 1
        points = []
        for i in range(num_points + 1):  # +1 для замыкания сектора
            current_azimuth = start_azimuth + (end_azimuth - start_azimuth) * i / num_points
            point = geodesic(meters=self.radius).destination((lat_center, lon_center), current_azimuth)
            points.append((point.longitude, point.latitude))

        # Замыкаем полигон, добавляя центр
        points.append((lon_center, lat_center))

        return points

    def contains_point(self, point):
        """
        Проверяет, находится ли точка внутри сектора.

        Параметры:
        - point: tuple (longitude, latitude) — координаты точки.

        Возвращает:
        - bool: True, если точка внутри сектора, иначе False.
        """
        return self.polygon.contains(Point(point))
