sc = satelliteScenario;

% 1200 km Earth orbit, 86.4 deg inclination, 588 sats in 12 planes. Phasing?
ow = walkerStar(sc, 1200e3+6378.14e3, 86.4, 588, 12, 1, Name="OneWeb");
satelliteScenarioViewer(sc);