#!/usr/bin/env python3

import math


class GPS2WP():
    def __init__(self):
        self.file_path = '/home/dev/gps_setting_ws/gps_trajectory/trajectory_20260116_100715.txt'
        self.save_path = '/home/dev/gps_setting_ws/gps_trajectory/wp_2.txt'
        
        self.gps, self.wp = [], []

    def read_gps(self):
        with open(self.file_path, 'r', encoding='utf-8') as f:
            for line in f:
                s = line.strip()
                if not s:
                    continue

                data = [d for d in s.split() if d]
                
                lat, lon = float(data[0]), float(data[1])
                self.gps.append((lat, lon))

    def haversine(self, lat0, lon0, lat, lon):
        r = 6371000.0 # meters
        
        dlat, dlon = math.radians(lat - lat0), math.radians(lon - lon0)
        lat0, lat = math.radians(lat0), math.radians(lat)
        a = math.sin(dlat/2)**2 + math.sin(dlon/2)**2 * math.cos(lat0) * math.cos(lat)
        return 2 * r * math.asin(math.sqrt(a))

    def convert_wp(self):
        self.wp.append(self.gps[0])
        prev = self.gps[0]

        for p in self.gps[1:]:
            if self.haversine(prev[0], prev[1], p[0], p[1]) >= 2.0:
                self.wp.append(p)
                prev = p

        if self.gps[-1] != self.wp[-1]:
            self.wp.append(self.gps[-1])

    def save_file(self):
        with open(self.save_path, 'w', encoding='utf-8') as f:
            for lat, lon in self.wp:
                f.write(f'{lat:.15f} {lon:.15f}\n')

        print("Way points saved")


def main():
    gps2wp = GPS2WP()

    gps2wp.read_gps()
    gps2wp.convert_wp()
    gps2wp.save_file()


if __name__ == "__main__":
    main()


