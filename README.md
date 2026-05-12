<h1>SR DEV Onstage 2026 Repository</h1>

<h3>Project Outline</h3>
<ol>
  <li>central</li>
    <ul>
      <li>Onstage_Master.py</li>
        <ul>
          <li>Robot, plant, ice, obstacle classes</li>
          <li>Object detection and location</li>
          <li>Target assignment logic</li>
          <li>Communication with robots</li>
          <li>Extensive testing logic with/without WiFi; image visualization</li>
        </ul>
      <li>Onstage_Rcoords.py</li>
        <ul>
          <li>AprilTag location, rotation, and ID detection</li>
          <li>Obstacle detection using HSV color mask</li>
          <li>Image annotation with AprilTag data for testing</li>
        </ul>
      <li>Onstage_WifiComms.py</li>
        <ul>
          <li>Wifi connection/disconnection and read/write</li>
        </ul>
      <li>CBF</li>
        <ul>
          <li>Helpers for computing whether or not a point is within a polygon and its distance to a given polygon boundary</li>
          <li>Control barrier function implementation - constraint creation and velocity calculation</li>
          <li>Logic for testing and visualizing CBF logic using matplotlib</li>
        </ul>
      <li>Onstage_Audio.py</li>
        <ul>
          <li>Management of audio channels to ensure no overlapping</li>
          <li>Pre-defined audio channels and audio filesn</li>
        </ul>
      <li>(obsolete)Onstage_pfield.py</li>
        <ul>
          <li>Implementation of potential field pathing</li>
          <li>Visualization of slope field</li>
        </ul>
    </ul>
  <li>ice_onstage</li>
    <ul>
      <li>ice_onstage.ino</li>
        <ul>
          <li>LED strip management logic</li>
          <li>Server implementation; receives WiFi communications and changes LEDs accordingly</li>
        </ul>
    </ul>
  <li>Cheese</li>
</ol>
