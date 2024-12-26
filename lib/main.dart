// import 'dart:typed_data';
import 'package:flutter/material.dart';
import 'dart:async';
import 'dart:math';

import 'package:flutter/services.dart';
import 'package:permission_handler/permission_handler.dart';
import 'package:intl/intl.dart';
import 'package:vector_math/vector_math_64.dart' hide Colors;
import "package:latlong2/latlong.dart";
import 'package:gps_tracker/gps_tracker.dart';
import 'package:gps_tracker_db/gps_tracker_db.dart';
import 'package:simple_sensor/simple_sensor.dart';

// final navigatorKey = GlobalKey<NavigatorState>();
/*
_listener is called when we get a GPS fix
initState contains listener functions for accelerometer etc
1) Get first GPS fix
2) Start calculating from accelerometer and orientation matrix
    on accelerometer event, get the orientation and normalise the acceleration
    calculate distance
3) On next GPS fix, print error between calculated and exact
 */

void main() {
  runApp(const MyApp());
}

class MyApp extends StatelessWidget {
  const MyApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(
        appBar: AppBar(
          title: const Text('Accuracy Test'),
        ),
        body: const MyStatefulApp(),
      ),
    );
  }
}

class MyStatefulApp extends StatefulWidget {
  const MyStatefulApp({super.key});

  @override
  MyAppState createState() => MyAppState();
}

class MyAppState extends State<MyStatefulApp> with WidgetsBindingObserver {
  String LISTENER_NAME = "com.moorwen.main";
  String _permissionStatus = 'Unknown status.';
  String _locationEnabled = 'Unknown location enabled.';
  String _location = 'Unknown location.';
  String _locationError = "Unknown error";
  String _numWalkTrackPoints = 'Unknown points.';
  String _distance = 'Unknown distance.';
  String _inDistance = 'Unknown distance';
  String _inLatLon = 'Unknown IN lat/lon';
  Timer? _timer;
  bool serviceStarted = false;
  bool tracking = false;

  Vector3 _accelerometer        = Vector3.zero();
  Vector3 _gyroscope            = Vector3.zero();
  Vector3 _magnetometer         = Vector3.zero();
  Vector3 _userAccelerometer   = Vector3.zero();
  Vector3 _orientation          = Vector3.zero();
  Vector3 _absoluteOrientation  = Vector3.zero();
  Vector3 _absoluteOrientation2 = Vector3.zero();
  double? _screenOrientation    = 0;
  int     _previousReportTime   = -1;
  double  _previousXSpeed       = 0.0;
  double  _previousYSpeed       = 0.0;
  double  _previousZSpeed       = 0.0;
  double  _distanceX            = 0.0;
  double  _distanceY            = 0.0;
  double  _distanceZ            = 0.0;
  double _latitude              = 0.0;
  double _longitude             = 0.0;
  double _accuracy              = 0.0;
  double _speed                 = 0.0;
  LatLng _prevLatLng            = LatLng(0.0, 0.0);
  bool _paused                   = false;

  static const String PROMPT_OK = "OK";
  
  static const double EARTH_RADIUS = 6378.137;

  ElevatedButton startServiceButton = const ElevatedButton(
    onPressed: null,
    child: Text('Start'),
  );
  ElevatedButton stopServiceButton = const ElevatedButton(
    onPressed: null,
    child: Text('Stop'),
  );
  ElevatedButton startTrackingButton = const ElevatedButton(
    onPressed: null,
    child: Text('Start Tracking'),
  );
  ElevatedButton stopTrackingButton = const ElevatedButton(
    onPressed: null,
    child: Text('Stop Tracking'),
  );
  ElevatedButton showDistanceButton = const ElevatedButton(
    onPressed: null,
    child: Text('Distance'),
  );
  ElevatedButton showNumPointsButton = const ElevatedButton(
    onPressed: null,
    child: Text('Num points'),
  );
  ElevatedButton showLocationButton = const ElevatedButton(
    onPressed: null,
    child: Text('Location'),
  );

  @override
  void initState() {
    super.initState();
    // IsFirstRun.isFirstCall().then((firstCall){
    //   int i = 3;
    // });
    WidgetsBinding.instance.addObserver(this);

    simpleSensor.gyroscope.listen((GyroscopeEvent event) {
      _gyroscope.setValues(event.x, event.y, event.z);
    });
    simpleSensor.accelerometer.listen((AccelerometerEvent event) {
      _accelerometer.setValues(event.x, event.y, event.z);
    });
    simpleSensor.userAccelerometer.listen((UserAccelerometerEvent event) {
      _userAccelerometer.setValues(event.x, event.y, event.z);
      calculateLocation(_absoluteOrientation, _userAccelerometer);
    });
    simpleSensor.magnetometer.listen((MagnetometerEvent event) {
      _magnetometer.setValues(event.x, event.y, event.z);
      var matrix = simpleSensor.getRotationMatrix(_accelerometer, _magnetometer);
      _absoluteOrientation2.setFrom(simpleSensor.getOrientation(matrix));
    });
    simpleSensor.isOrientationAvailable().then((available) {
      if (available) {
        simpleSensor.orientation.listen((OrientationEvent event) {
          _orientation.setValues(event.yaw, event.pitch, event.roll);
        });
      }
    });
    simpleSensor.absoluteOrientation.listen((AbsoluteOrientationEvent event) {
      _absoluteOrientation.setValues(event.yaw, event.pitch, event.roll);
    });
    simpleSensor.screenOrientation.listen((ScreenOrientationEvent event) {
      _screenOrientation = event.angle;
    });
  }

  Matrix3 rotationMatrixFromOrientationVector(Vector3 o) {
    Matrix4 m = Matrix4.zero();
    simpleSensor.getOrientation(m);

    return Matrix3(
        cos(o.x)*cos(o.z) - sin(o.x)*cos(o.y)*sin(o.z),
        sin(o.x)*cos(o.z) + cos(o.y)*cos(o.x)*sin(o.z),
        sin(o.y)*sin(o.z),
        -cos(o.x)*sin(o.z) - cos(o.y)*sin(o.x)*cos(o.z),
        -sin(o.x)*sin(o.z) + cos(o.y)*cos(o.x)*cos(o.z),
        sin(o.y)*cos(o.z),
        sin(o.x)*sin(o.y),
        -cos(o.x)*sin(o.y),
        cos(o.y));
  }

  // Calculate distance travelled and final speed from acceleration, initial speed and time.
  // Acceleration is m/s**2
  // Speed is m/s
  // Time is in milliseconds
  // Output distance is in metres
  List<double> calculateDistanceAndSpeed(double accel, double initialSpeed, int time) {
    var distanceAndSpeed = <double>[0.0,0.0];
    double finalSpeed = initialSpeed + (accel*time)/1000.0;
    distanceAndSpeed[0] = finalSpeed;
    distanceAndSpeed[1] = (initialSpeed + finalSpeed)*0.5*(time/1000.0);

    return distanceAndSpeed;
  }

  void calculateLocation(Vector3 orientation, Vector3 acceleration) {
    if (_previousReportTime >= 0) {
      int now = DateTime.now().millisecondsSinceEpoch;
      int interval = now - _previousReportTime;
      Matrix3 transformer = rotationMatrixFromOrientationVector(
          Vector3(radians(orientation.x), radians(orientation.y),
              radians(orientation.z))
      );
      transformer.transform(acceleration);

      List<double> distAndSpeed = calculateDistanceAndSpeed(
          acceleration.x, _previousXSpeed, interval);
      _distanceX = _distanceX + distAndSpeed[0];
      _previousXSpeed = distAndSpeed[1];
      distAndSpeed =
          calculateDistanceAndSpeed(acceleration.y, _previousYSpeed, interval);
      _distanceY = _distanceY + distAndSpeed[0];
      _previousYSpeed = distAndSpeed[1];
      distAndSpeed =
          calculateDistanceAndSpeed(acceleration.z, _previousZSpeed, interval);
      _distanceZ = distAndSpeed[0];
      _previousZSpeed = _distanceZ + distAndSpeed[1];

      _previousReportTime = now;
      var f = NumberFormat("#.#######", "en_UK");
      LatLng estimate = newPosition(_prevLatLng, _distanceX, _distanceY);

      if (!_paused) {
        setState(() {
          _inLatLon = "in lat ${f.format(estimate.latitude)} lon ${f.format(
              estimate.longitude)}";
          _inDistance =
          "in X ${f.format(_distanceX)} Y ${f.format(_distanceY)}";
        });
      }
    }
  }

  // Calculate the new lat/lon from the current lat/lon and x/y distance (x - NorthSouth, y - EastWest)
  // https://stackoverflow.com/questions/7477003/calculating-new-longitude-latitude-from-old-n-meters
  // Latitude:
  //    var earth = 6378.137,  //radius of the earth in kilometer
  //       pi = Math.PI,
  //       m = (1 / ((2 * pi / 360) * earth)) / 1000;  //1 meter in degree`
  //    var new_latitude = latitude + (your_meters * m);
  // Longitude:
  //   var earth = 6378.137,  //radius of the earth in kilometer
  //      pi = Math.PI,
  //      cos = Math.cos,
  //      m = (1 / ((2 * pi / 360) * earth)) / 1000;  //1 meter in degree
  //   var new_longitude = longitude + (your_meters * m) / cos(latitude * (pi / 180));
  LatLng newPosition(LatLng currentPosition, double x, double y) {
    var oneMetre  = (1.0 / ((2.0 * pi / 360) * EARTH_RADIUS)) / 1000;
    var newLat = currentPosition.latitude + (x*oneMetre);
    var newLon = currentPosition.longitude + (x*oneMetre)/(cos(currentPosition.longitudeInRad));
    return LatLng(newLat, newLon);
  }

  Future<void> showMessage(String title, String message) async {
    return showDialog<void>(
      context: context,
      barrierDismissible: false, // user must tap button!
      builder: (BuildContext context) {
        return AlertDialog(
          title: Text(title),
          content: SingleChildScrollView(
            child: ListBody(
              children: <Widget>[
                Text(message),
              ],
            ),
          ),
          actions: <Widget>[
            TextButton(
              child: const Text(PROMPT_OK),
              onPressed: () {
                Navigator.of(context, rootNavigator: true).pop();
              },
            ),
          ],
        );
      },
    );
  }

  void _actionsWhenPermissionGranted() {
    setState(() {
      startServiceButton = ElevatedButton(
        onPressed: _startService,
        child: const Text('Start'),
      );
    });
  }

  void _actionsWhenServiceStarted() {
    setState(() {
      startServiceButton = const ElevatedButton(
        onPressed: null,
        child: Text('Start'),
      );
      stopServiceButton = ElevatedButton(
        onPressed: _stopService,
        child: const Text('Stop'),
      );
      startTrackingButton = ElevatedButton(
        onPressed: _startTracking,
        child: const Text('Start Tracking'),
      );
    });
  }

  void _actionsWhenServiceStopped() {
    setState(() {
      startServiceButton = ElevatedButton(
        onPressed: _startService,
        child: const Text('Start'),
      );
      stopServiceButton = const ElevatedButton(
        onPressed: null,
        child: Text('Stop'),
      );
      startTrackingButton = const ElevatedButton(
        onPressed: null,
        child: Text('Start Tracking'),
      );
    });
  }

  void _actionsWhenTrackingStarted() {
    setState(() {
      stopServiceButton = const ElevatedButton(
        onPressed: null,
        child: Text('Stop'),
      );
      startTrackingButton = const ElevatedButton(
        onPressed: null,
        child: Text('Start Tracking'),
      );
      stopTrackingButton = ElevatedButton(
        onPressed: _stopTracking,
        child: const Text('Stop Tracking'),
      );
      showDistanceButton = ElevatedButton(
        onPressed: _getDistance,
        child: const Text('Distance'),
      );
      showNumPointsButton = ElevatedButton(
        onPressed: _getNumWalkTrackPoints,
        child: const Text('Num points'),
      );
      showLocationButton = ElevatedButton(
        onPressed: _getLocation,
        child: const Text('Location'),
      );
    });
  }

  void _actionsWhenTrackingStopped() {
    setState(() {
      stopServiceButton = ElevatedButton(
        onPressed: _stopService,
        child: const Text('Stop'),
      );
      startTrackingButton = ElevatedButton(
        onPressed: _startTracking,
        child: const Text('Start Tracking'),
      );
      stopTrackingButton = const ElevatedButton(
        onPressed: null,
        child: Text('Stop Tracking'),
      );
      showDistanceButton = const ElevatedButton(
        onPressed: null,
        child: Text('Distance'),
      );
      showNumPointsButton = const ElevatedButton(
        onPressed: null,
        child: Text('Num points'),
      );
      showLocationButton = const ElevatedButton(
        onPressed: null,
        child: Text('Location'),
      );
    });
  }

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(
        body: Center(
          child: SingleChildScrollView(
            child: Column(
              children: <Widget>[
                Row(children: <Widget>[
                  const Spacer(flex: 10),
                  ElevatedButton(
                    onPressed: _checkPermissions,
                    child: const Text('Permissions'),
                  ),
                  const Spacer(flex: 10),
                ]),
                Text(_permissionStatus),
                Row(children: <Widget>[
                  const Spacer(flex: 10),
                  ElevatedButton(
                    onPressed: _requestPermissions,
                    child: const Text('Request permissions'),
                  ),
                  const Spacer(flex: 10),
                ]),
                ElevatedButton(
                  onPressed: _getLocationEnabled,
                  child: const Text('Get Location Enabled'),
                ),
                Text(_locationEnabled),
                Row(children: <Widget>[
                  const Spacer(flex: 10),
                  startServiceButton,
                  const Spacer(),
                  stopServiceButton,
                  const Spacer(flex: 10),
                ]),
                Row(children: <Widget>[
                  const Spacer(flex: 10),
                  startTrackingButton,
                  const Spacer(),
                  stopTrackingButton,
                  const Spacer(flex: 10),
                ]),
                Row(children: <Widget>[
                  const Spacer(flex: 10),
                  showDistanceButton,
                  const Spacer(),
                  showNumPointsButton,
                  const Spacer(),
                  showLocationButton,
                  const Spacer(flex: 10),
                ]),
                ElevatedButton(
                  onPressed: () {_paused = !_paused;},
                  child: const Text('Pause display'),
                ),
                Text(_numWalkTrackPoints),
                Text(_location),
                Text(_distance),
                Text(_inDistance),
                Text(_inLatLon),
                Text(_locationError),
              ],
            ),
          ),
        ),
      ),
    );
  }

  Future<void> _checkPermissions() async {
    int status = await GpsTracker.getCurrentLocationPermissions();
    String statusMsg = "Permission status: unknown";
    switch (status) {
      case GpsTracker.GRANTED:
        statusMsg = "Permission status: granted ($status)";
        break;
      case GpsTracker.LOCATION_OFF:
        statusMsg = "Permission status: location off ($status)";
        break;
      case GpsTracker.INACCURATE_LOCATION:
        statusMsg = "Permission status: inaccurate location ($status)";
        break;
      case GpsTracker.DENIED:
        statusMsg = "Permission status: denied ($status)";
        break;
      case GpsTracker.PARTLY_DENIED:
        statusMsg = "Permission status: partly denied ($status)";
        break;
      case GpsTracker.PERMANENTLY_DENIED:
        statusMsg = "Permission status: permanently denied ($status)";
        break;
      default:
        statusMsg = "Permission status: unknown ($status)";
        break;
    }
    setState(() {
      _permissionStatus = statusMsg;
    });
  }

  Future<void> _requestPermissions() async {
    int status = await GpsTracker.getCurrentLocationPermissions();
    switch (status) {
      case GpsTracker.GRANTED:
        _actionsWhenPermissionGranted();
        break;

      case GpsTracker.DENIED:
        while (status == GpsTracker.DENIED) {
          status = await GpsTracker.requestLocationPermissions();
        }
        if (status == GpsTracker.GRANTED) {
          _actionsWhenPermissionGranted();
        } else if (status == GpsTracker.PERMANENTLY_DENIED) {
          showDialog(
              context: context,
              builder: (BuildContext context) {
                return const LocationRequiredDialog();
              }).then((val) {
            // Navigator.pop(context);
            if (val) {
              openAppSettings();
            }
          });
        }
        break;
      case GpsTracker.LOCATION_OFF:
      case GpsTracker.INACCURATE_LOCATION:
      case GpsTracker.PARTLY_DENIED:
      case GpsTracker.PERMANENTLY_DENIED:
        showDialog(
            context: context,
            builder: (BuildContext context) {
              return const LocationRequiredDialog();
            }).then((val) {
          // Navigator.pop(context);
          if (val) {
            openAppSettings();
          }
        });
        break;
      default:
        break;
    }
  }

  Future<void> _startService() async {
    if (!serviceStarted) {
      GpsTracker.addGpsListener(_listener);
      await GpsTracker.start(
        title: "GPS Tracker",
        text: "Text",
        subText: "Subtext",
        ticker: "Ticker",
      );
      serviceStarted = true;
    }
    _actionsWhenServiceStarted();
  }

  Future<void> _stopService() async {
    if (serviceStarted) {
      GpsTracker.removeGpsListener(_listener);
      GpsTracker.stop();
      serviceStarted = false;
    }
    _actionsWhenServiceStopped();
  }

  Future<void> _getLocationEnabled() async {
    String locationEnabled;
    try {
      final int result = await GpsTracker.locationEnabled;
      locationEnabled = 'Location enabled $result.';
    } on PlatformException catch (e) {
      locationEnabled = "Failed to get location enabled: '${e.message}'.";
    }

    setState(() {
      _locationEnabled = locationEnabled;
    });
  }

  Future<void> _startTracking() async {
    if (serviceStarted && !tracking) {
      var db = await DatabaseHelper.getDatabaseHelper();
      DateTime now = DateTime.now();
      String formattedDate = DateFormat("yyyy-MM-dd HH:mm:ss").format(now);
      await db.addWalk(formattedDate);
      GpsTracker.startTracking(formattedDate);
      tracking = true;
//    startTimer();
    }
    _actionsWhenTrackingStarted();
  }

  void _listener(dynamic o) {
    print("MAIN - GPS tracker update"); // - reason $reason status $status lat $lat lon $lon");
    // print("type "  + o.runtimeType.toString());
    // Map retval = o as Map;
    // print("retval type "  + retval.runtimeType.toString());
    // retval.forEach((k,v) {
    //   print("k type " + k.runtimeType.toString() + " v type " + v.runtimeType.toString());
    //   print("k $k v $v");
    // });
    Map map = o as Map;
    var reason = map["reason"];
    var fixValid = map["fix_valid"] as bool;
    if (reason == "COORDINATE_UPDATE") {
      _latitude = map["latitude"] as double;
      _longitude = map["longitude"] as double;
      _accuracy = map["accuracy"] as double;
      _speed = map["speed"] as double;
      print(
          "COORDINATE UPDATE - latitude $_latitude longitude $_longitude speed $_speed accuracy $_accuracy fix_valid $fixValid");

      String location;
      String error = "Unknown error";
      try {
        var f = NumberFormat("#.#######", "en_UK");
        location = "Lat ${f.format(_latitude)} long ${f.format(_longitude)}";
      } on PlatformException catch (e) {
        location = "Failed to get location: '${e.message}'.";
      }
      if (_previousReportTime >= 0) {
        var f = NumberFormat("#.#######", "en_UK");
        LatLng estimate = newPosition(_prevLatLng, _distanceX, _distanceY);
        error = "error lat ${f.format(estimate.latitude - _latitude)} lon ${f.format(estimate.longitude - _longitude)}";
      }
      if (!_paused) {
        setState(() {
          _location = location;
          _locationError = error;
        });
      }
      if (_previousReportTime < 0) {
        _previousReportTime = DateTime
            .now()
            .millisecondsSinceEpoch;
      }
      _prevLatLng = LatLng(_latitude, _longitude);
      _distanceX = 0.0;
      _distanceY = 0.0;
      _distanceZ = 0.0;

    } else {
      print("FIX UPDATE - fix valid $fixValid");
    }
  }

  Future<void> _stopTracking() async {
    if (serviceStarted && tracking) {
      GpsTracker.stopTracking();
      if (_timer != null) {
        _timer!.cancel();
        _timer = null;
      }
      tracking = false;
    }
    _actionsWhenTrackingStopped();
  }

  Future<void> _getLocation() async {
    String location;
    try {
      final result = await GpsTracker.getLocation();
      var lat = result!.first;
      var long = result.last;
      var f = NumberFormat("#.#######", "en_UK");
      location = "Lat ${f.format(lat)} long ${f.format(long)}";
    } on PlatformException catch (e) {
      location = "Failed to get location: '${e.message}'.";
    }
    setState(() {
      _location = location;
    });
  }

  Future<void> _getNumWalkTrackPoints() async {
    String numWalkTrackPoints;
    try {
      final result = await GpsTracker.getNumWalkTrackPoints();
      numWalkTrackPoints = "There are $result track points";
    } on PlatformException catch (e) {
      numWalkTrackPoints = "Failed to get num track points: '${e.message}'.";
    }
    setState(() {
      _numWalkTrackPoints = numWalkTrackPoints;
    });
  }

  Future<void> _getWalkTrackPoints() async {
    try {
      final numPoints = await GpsTracker.getNumWalkTrackPoints();
      final result = await GpsTracker.getWalkTrackPoints(0, numPoints);
      String resultType = result.runtimeType.toString();
      int numEntries = result.length;
      for (final latLong in result) {
        var currentElement = latLong;
        resultType = currentElement.runtimeType.toString();
        int numElements = latLong.length;
        var lat = latLong[0];
        var lon = latLong[1];
      }
    } on PlatformException {}
  }

  Future<void> _getDistance() async {
    String distance;
    try {
      final result = await GpsTracker.getDistance();
      final resultToPrint = result.toInt();
      distance = "Travelled $resultToPrint metres";
    } on PlatformException catch (e) {
      distance = "Failed to get distance: '${e.message}'.";
    }
    setState(() {
      _distance = distance;
    });
  }

  void startTimer() {
    const oneSec = Duration(seconds: 1);
    _timer = Timer.periodic(
      oneSec,
      (Timer timer) {
// Put code to read stuff here
        _getNumWalkTrackPoints();
        _getLocation();
        _getDistance();
        _getWalkTrackPoints();
      },
    );
  }

  @override
  void dispose() {
    _timer!.cancel();
    _timer = null;
    WidgetsBinding.instance.removeObserver(this);
    super.dispose();
  }

  @override
  void didChangeAppLifecycleState(AppLifecycleState lifecycleState) {
    if (lifecycleState == AppLifecycleState.resumed) {
      GpsTracker.checkForLocationPermissionChanges();
    } else if (lifecycleState == AppLifecycleState.paused) {
      int i = 3;
    }
  }
}

class LocationRequiredDialog extends StatefulWidget {

  const LocationRequiredDialog({super.key});
  @override
  _LocationRequiredDialogState createState() => _LocationRequiredDialogState();
}

class _LocationRequiredDialogState extends State<LocationRequiredDialog> {
  @override
  Widget build(BuildContext context) {
    return AlertDialog(
      title: const Text('Location Status'),
      content: const SingleChildScrollView(
        child: ListBody(
          children: <Widget>[
            Text(
                'In order for this application to function, it requires location tracking to be enabled, precise accuracy to be turned on, and access to all location functions. Press the SETTINGS button to open the settings window then enable all these features to continue.'),
          ],
        ),
      ),
      actions: <Widget>[
        TextButton(
          child: const Text('Settings'),
          onPressed: () {
            Navigator.of(context).pop(true);
          },
        ),
        TextButton(
          child: const Text('Disable'),
          onPressed: () {
            Navigator.of(context).pop(false);
          },
        ),
      ],
    );
  }
}