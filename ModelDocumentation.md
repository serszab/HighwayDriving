# Highway driving

Author: Szabolcs Sergyan

## Path planning policy

In my path planning I used the following concept:
- If the lane of my car was free in front of my car I tried to drive with the highest possible speed. In order to do that if the speed of my car was less than 49.5 mph/h I increased the speed by 0.224 mph/h.
```
if (!too_close && ref_vel < 49.5) {
    ref_vel += 0.224;
}
```
- If a car was too close in the lane of my car to my car I decreased the speed and then investigated whether lane changing is possible. In speed decreasing I considered the speed of the other car as well.
```
if (check_car_s > car_s && check_car_s - car_s < 30) {
    ref_vel -= (ref_vel - check_speed) / 30.0;
}
```
- In lane changing investigation I checked first the right lane and then the left lane. If the investigated lane was free I made lane change. A lane was considered free if in front of my car 20 meters was free and behind my car 10 meters was free. I also investigated that whether an other car comes behing my car with greater speed than my car had.
```
bool isLaneFree(const int laneNumber, const double myCarS, const double mySpeed, const vector<vector<double>>& sensorFusionData)
{
  for (size_t i = 0; i < sensorFusionData.size(); ++i) {
    const int laneOfOtherCar = static_cast<int>(sensorFusionData[i][6]) / 4;
    if (laneOfOtherCar == laneNumber) {
      const double otherCarS = sensorFusionData[i][5];
      if ((myCarS <= otherCarS && otherCarS <= myCarS + 20) || (myCarS - 10 < otherCarS && otherCarS < myCarS)) {
        return false;
      }
      const double otherVelocityX = sensorFusionData[i][3];
      const double otherVelocityY = sensorFusionData[i][4];
      const double otherVelocity = sqrt(otherVelocityX * otherVelocityX + otherVelocityY * otherVelocityY);
      if (myCarS - 20 < otherCarS && otherCarS < myCarS - 10 && mySpeed < otherVelocity) {
        return false;
      }
    }
  }
  return true;
}
```
## Trajectory generation

In trajectory generation I used the code what was presented in Q&A session. I only modified the code below when lane changing happened. In order to generate a smoother trajectory I used 45 meters as the distance the car should arrive to the target lane.
```
const int firstIncrease = (lane == static_cast<int>(car_d) / 4) ? 30 : 45;
vector<double> next_wp0 = getXY(car_s + firstIncrease, 2 + 4 * lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
vector<double> next_wp1 = getXY(car_s + 60, 2 + 4 * lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
vector<double> next_wp2 = getXY(car_s + 90, 2 + 4 * lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
```

## Experimental results
My car was able to run in the simulator without incident for 30 minutes. The total distance was 21.34 miles so the average speed of my car was 42.68 mph/h.