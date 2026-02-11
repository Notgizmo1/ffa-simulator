"""
Mission Calculator - Phase 3.2
Haversine distance, bearing, ETA, and mission progress calculations.
"""

import math
import logging

logger = logging.getLogger(__name__)


class MissionCalculator:
    """Static utility class for mission progress calculations"""

    EARTH_RADIUS_M = 6371000  # Earth radius in meters

    @staticmethod
    def haversine_distance(lat1, lon1, lat2, lon2):
        """
        Calculate great-circle distance between two GPS points.

        Args:
            lat1, lon1: First point (decimal degrees)
            lat2, lon2: Second point (decimal degrees)

        Returns:
            Distance in meters
        """
        lat1_r = math.radians(lat1)
        lat2_r = math.radians(lat2)
        dlat = math.radians(lat2 - lat1)
        dlon = math.radians(lon2 - lon1)

        a = (math.sin(dlat / 2) ** 2 +
             math.cos(lat1_r) * math.cos(lat2_r) * math.sin(dlon / 2) ** 2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

        return MissionCalculator.EARTH_RADIUS_M * c

    @staticmethod
    def calculate_bearing(lat1, lon1, lat2, lon2):
        """
        Calculate initial bearing from point 1 to point 2.

        Args:
            lat1, lon1: Origin point (decimal degrees)
            lat2, lon2: Destination point (decimal degrees)

        Returns:
            Bearing in degrees (0-360, 0=North, 90=East)
        """
        lat1_r = math.radians(lat1)
        lat2_r = math.radians(lat2)
        dlon = math.radians(lon2 - lon1)

        x = math.sin(dlon) * math.cos(lat2_r)
        y = (math.cos(lat1_r) * math.sin(lat2_r) -
             math.sin(lat1_r) * math.cos(lat2_r) * math.cos(dlon))

        bearing = math.atan2(x, y)
        return (math.degrees(bearing) + 360) % 360

    @staticmethod
    def bearing_to_cardinal(bearing):
        """
        Convert bearing degrees to cardinal direction string.

        Args:
            bearing: Degrees 0-360

        Returns:
            Cardinal direction string (e.g., "N", "NE", "E", etc.)
        """
        directions = ["N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE",
                       "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"]
        index = round(bearing / 22.5) % 16
        return directions[index]

    @staticmethod
    def calculate_eta(distance_m, groundspeed_ms):
        """
        Calculate estimated time of arrival.

        Args:
            distance_m: Distance in meters
            groundspeed_ms: Groundspeed in meters/second

        Returns:
            ETA in seconds, or None if groundspeed is zero/invalid
        """
        if groundspeed_ms is None or groundspeed_ms <= 0.5:
            return None
        return distance_m / groundspeed_ms

    @staticmethod
    def calculate_total_mission_distance(waypoints):
        """
        Calculate total mission distance across all waypoint legs.

        Args:
            waypoints: List of Waypoint objects with .lat, .lon attributes

        Returns:
            Total distance in meters
        """
        if len(waypoints) < 2:
            return 0.0

        total = 0.0
        for i in range(len(waypoints) - 1):
            total += MissionCalculator.haversine_distance(
                waypoints[i].lat, waypoints[i].lon,
                waypoints[i + 1].lat, waypoints[i + 1].lon
            )
        return total

    @staticmethod
    def calculate_remaining_distance(current_lat, current_lon,
                                     current_wp_index, waypoints):
        """
        Calculate remaining mission distance from current position.

        Distance = (current pos -> next WP) + (all subsequent WP legs)

        Args:
            current_lat, current_lon: Current aircraft position
            current_wp_index: Index into waypoints list for the NEXT target WP
            waypoints: List of Waypoint objects

        Returns:
            Remaining distance in meters
        """
        if not waypoints or current_wp_index >= len(waypoints):
            return 0.0

        # Distance from current position to next waypoint
        next_wp = waypoints[current_wp_index]
        remaining = MissionCalculator.haversine_distance(
            current_lat, current_lon, next_wp.lat, next_wp.lon
        )

        # Add all subsequent waypoint-to-waypoint legs
        for i in range(current_wp_index, len(waypoints) - 1):
            remaining += MissionCalculator.haversine_distance(
                waypoints[i].lat, waypoints[i].lon,
                waypoints[i + 1].lat, waypoints[i + 1].lon
            )

        return remaining

    @staticmethod
    def format_time(seconds):
        """
        Format seconds into HH:MM:SS string.

        Args:
            seconds: Time in seconds (float or int), or None

        Returns:
            Formatted string "HH:MM:SS" or "---" if None/invalid
        """
        if seconds is None or seconds < 0:
            return "---"

        seconds = int(seconds)
        hours = seconds // 3600
        minutes = (seconds % 3600) // 60
        secs = seconds % 60
        return f"{hours:02d}:{minutes:02d}:{secs:02d}"

    @staticmethod
    def format_distance(meters):
        """
        Format distance with appropriate units.

        Args:
            meters: Distance in meters

        Returns:
            Formatted string (e.g., "245m" or "1.8km")
        """
        if meters >= 1000:
            return f"{meters / 1000:.1f}km"
        return f"{meters:.0f}m"
