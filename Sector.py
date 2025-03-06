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
        Возвращает точку пересечения LineString с осями сектора.

        Параметры:
        - line: LineString — линия для проверки.

        Возвращает:
        - Point или None: точка пересечения с осями или None, если пересечения нет.
        """
        # Определяем центральную ось как LineString от центра до точки на радиусе по азимуту
        lon_center, lat_center = self.center
        central_axis_end = geodesic(meters=self.radius).destination((lat_center, lon_center), self.azimuth)
        central_axis = LineString([(lon_center, lat_center), (central_axis_end.longitude, central_axis_end.latitude)])

        # Определяем левую и правую оси
        left_axis = None
        right_axis = None

        if self.angle > 180:
            left_axis_end = geodesic(meters=self.radius).destination((lat_center, lon_center), self.azimuth - 90)
            right_axis_end = geodesic(meters=self.radius).destination((lat_center, lon_center), self.azimuth + 90)
            left_axis = LineString([(lon_center, lat_center), (left_axis_end.longitude, left_axis_end.latitude)])
            right_axis = LineString([(lon_center, lat_center), (right_axis_end.longitude, right_axis_end.latitude)])

        # Список осей для проверки
        axes = []
        if central_axis is not None:
            axes.append(central_axis)
        if left_axis is not None:
            axes.append(left_axis)
        if right_axis is not None:
            axes.append(right_axis)

        # Список точек пересечения
        intersection_points = []

        for axis in axes:
            if axis.intersects(line):
                intersection = axis.intersection(line)
                if not intersection.is_empty:
                    if intersection.geom_type == 'Point':
                        intersection_points.append(intersection)
                    elif intersection.geom_type == 'MultiPoint':
                        intersection_points.extend(intersection.geoms)
                    else:
                        # Если пересечение — отрезок или другая геометрия, игнорируем
                        continue

        if not intersection_points:
            return None

        # Находим точку, ближайшую к центру
        center_point = Point(lon_center, lat_center)
        nearest_point = min(intersection_points, key=lambda p: p.distance(center_point))

        return nearest_point
