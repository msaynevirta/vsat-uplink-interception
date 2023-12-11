%% OneWeb visualisation

%startTime=datetime("28-Feb-2023 20:30:00");
%endTime=datetime("28-Feb-2023 21:32:25");

startTime=datetime("11-Dec-2023 11:00:00"); % oneweb 2023-12-11
endTime=datetime("11-Dec-2023 11:30:00"); % oneweb 2023-12-11

%startTime=datetime("11-Dec-2023 11:00:00"); % starlink 2023-12-11
%endTime=datetime("11-Dec-2023 11:30:00"); % starlink 2023-12-11
sampleTime=60;

sc = satelliteScenario(startTime, endTime, sampleTime);

%% 1200 km Earth orbit, 86.4 deg inclination, 588 sats in 12 planes. Phasing?
%ow = walkerStar(sc, 1200e3+6378.14e3, 86.4, 588, 12, 1, Name="OneWeb");

utKajaani = groundStation(sc, 64.2, 27.7, Name="Kajaani");
serverEspoo = groundStation(sc, 60.2, 24.8, Name="Espoo");
serverWormerveer = groundStation(sc, 52.5, 4.8, Name="Wormerveer");
popLondon = groundStation(sc, 51.5, -0.2, Name="London");

set(utKajaani, ShowLabel=false);
set(utKajaani, MarkerColor="Red");
set(serverEspoo, ShowLabel=false);
set(serverEspoo, MarkerColor="Green");
set(serverWormerveer, ShowLabel=false);
set(serverWormerveer, MarkerColor="Green");
set(popLondon, ShowLabel=false);
set(popLondon, MarkerColor="Yellow");

constellation = satellite(sc, "data/processed/oneweb_2023-03-01.tle");

%constellation = satellite(sc, "oneweb.tle");
%constellation = satellite(sc, "starlink.tle");

set(constellation(1:length(constellation)), ShowLabel=false);
set(constellation(1:length(constellation)), MarkerSize=3);
set(constellation(1:length(constellation)).Orbit, LineWidth=1);
%set(constellation(1:length(constellation)).Orbit, LineColor="none");

viewer3D = satelliteScenarioViewer(sc, Basemap="bluegreen", ShowDetails=false);

%% Camera position
campos(viewer3D, 40, 18);