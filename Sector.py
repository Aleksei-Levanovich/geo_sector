from geopy.distance import geodesic
from shapely.geometry import Polygon, Point
from shapely.geometry.linestring import LineString


class Sector:
    """
    Класс для представления сектора круга.

    Параметры:
    - center: tuple (longitude, latitude) — координаты центра сектора.
    - azimuth: float — азимут центральной оси сектора (в градусах).
    - angle: float — угол сектора (в градусах).
    - radius: float — длина радиуса сектора (в метрах).
    """

    def __init__(self, center: tuple[float, float], azimuth: float, angle: int, radius: float):
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
        num_points = self.angle
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

    def intersects_line(self, line: LineString):
        """
        Проверяет, пересекает ли LineString сектор.

        Параметры:
        - line: LineString — линия для проверки пересечения.

        Возвращает:
        - bool: True, если линия пересекает сектор, иначе False.
        """
        return self.polygon.intersects(line)

    def get_closest_intersection_point(self, line: LineString):
        """
        Находит ближайшую точку на пересечении LineString с сектором к центру.

        Возвращает:
        - Point или None: ближайшая точка к центру на пересечении или None, если пересечения нет.
        """
        if not self.polygon.intersects(line):
            return None

        center_point = Point(self.center)
        intersection = self.polygon.intersection(line)

        if intersection.is_empty:
            return None

        # Функция для поиска ближайшей точки на геометрии
        def find_closest_on_geometry(geom):
            if geom.geom_type == 'Point':
                return geom
            elif geom.geom_type == 'LineString':
                project_t = geom.project(center_point, normalized=True)
                return geom.interpolate(project_t, normalized=True)
            elif geom.geom_type in ['MultiPoint', 'MultiLineString']:
                geometry_closest_point = None
                min_distance = float('inf')
                for sub_geom in geom.geoms:
                    point = find_closest_on_geometry(sub_geom)
                    dist = point.distance(center_point)
                    if dist < min_distance:
                        min_distance = dist
                        geometry_closest_point = point
                return geometry_closest_point
            return None

        closest_point = find_closest_on_geometry(intersection)

        # Если пересечение есть, но точка не найдена, проверяем всю линию
        if not closest_point:
            t = line.project(center_point, normalized=True)
            candidate_point = line.interpolate(t, normalized=True)
            if self.polygon.intersects(Point(candidate_point)):  # Проверяем пересечение с точкой
                return candidate_point

        return closest_point

    def get_axis_intersection_point(self, line: LineString):
        """
        Возвращает точку пересечения LineString с центральной осью сектора.

        Параметры:
        - line: LineString — линия для проверки.

        Возвращает:
        - Point или None: точка пересечения с центральной осью или None, если пересечения нет.
        """
        # Определяем центральную ось как LineString от центра до точки на радиусе по азимуту
        lon_center, lat_center = self.center
        axis_end = geodesic(meters=self.radius).destination((lat_center, lon_center), self.azimuth)
        axis = LineString([(lon_center, lat_center), (axis_end.longitude, axis_end.latitude)])

        # Проверяем пересечение линии с осью
        if not axis.intersects(line):
            return None

        # Находим точку пересечения
        intersection = axis.intersection(line)

        if intersection.is_empty:
            return None

        if intersection.geom_type == 'Point':
            return intersection
        elif intersection.geom_type == 'MultiPoint':
            # Если несколько точек пересечения, возвращаем ближайшую к центру
            return min(intersection.geoms, key=lambda p: p.distance(Point(self.center)))
        else:
            # Если пересечение — отрезок или другая геометрия, возвращаем None
            return None
