% The clustering implementation above uses the following steps:
% If the detections are from the same sensor, then loop through every single detection point and measure the Euclidean distance between all of them.
% Initialize a vector to hold the detection clusters.
% Keep running the loop until the detection list is empty.

function detectionClusters = clusterDetections(detections, vehicleSize)
N = numel(detections);
distances = zeros(N);
for i = 1:N
    for j = i + 1: N
        if detections{i}.SensorIndex == dete
% The while loop above consists of the following steps:

% Pick the first detection in the checklist and check for its clustering neighbors.
% If the distance between the first pick and remaining detections is less than the vehicle size, then group those detections and their respective radar sensor measurements, including range and velocity.
% For the group, take the mean of the range and velocity measurements.

% Note: the radar measurement vector has 6 values - where range and velocity for x and y coordinates reside at indices 1,2, 4, and 5: [x, y, -, Vx, Vy, -]

% Create a new Cluster ID. Then, assign all the group detections to the same ID.
% Further, assign cluster, the mean range, and velocity.
% In the end, delete from the list the detections which have already been assigned to a cluster.
% Keep repeating the process until the detection list is empty.

