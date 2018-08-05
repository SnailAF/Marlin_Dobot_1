void inverse_kinematics(const float raw[XYZ]) {

    static float C2, S2, SK1, SK2, THETA, PSI;

    float sx = raw[X_AXIS] - SCARA_OFFSET_X,  // 将SCARA翻译成标准X Y  Translate SCARA to standard X Y
          sy = raw[Y_AXIS] - SCARA_OFFSET_Y;  // 与比例因子。  With scaling factor.

    if (L1 == L2)
      C2 = HYPOT2(sx, sy) / L1_2_2 - 1;
    else
      C2 = (HYPOT2(sx, sy) - (L1_2 + L2_2)) / (2.0 * L1 * L2);

    S2 = SQRT(1 - sq(C2));

    // Unrotated Arm1 plus rotated Arm2 gives the distance from Center to End
    // 未旋转的Arm1+旋转的Arm2给出了从中心到末端的距离
    SK1 = L1 + L2 * C2;

    // Rotated Arm2 gives the distance from Arm1 to Arm2
    // 旋转的Arm2提供了从Arm1到Arm2的距离
    SK2 = L2 * S2;

    // Angle of Arm1 is the difference between Center-to-End angle and the Center-to-Elbow
    // Arm1是中心到端角和中心到肘之间的区别
    THETA = ATAN2(SK1, SK2) - ATAN2(sx, sy);

    // Angle of Arm2
    PSI = ATAN2(S2, C2);

    delta[A_AXIS] = DEGREES(THETA);        //theta是支撑臂角 theta is support arm angle
    delta[B_AXIS] = DEGREES(THETA + PSI);  // 等于子臂角（反向马达）equal to sub arm angle (inverted motor)
    delta[C_AXIS] = raw[Z_AXIS];

    /*
      DEBUG_POS("SCARA IK", raw);
      DEBUG_POS("SCARA IK", delta);
      SERIAL_ECHOPAIR("  SCARA (x,y) ", sx);
      SERIAL_ECHOPAIR(",", sy);
      SERIAL_ECHOPAIR(" C2=", C2);
      SERIAL_ECHOPAIR(" S2=", S2);
      SERIAL_ECHOPAIR(" Theta=", THETA);
      SERIAL_ECHOLNPAIR(" Phi=", PHI);
    //*/
  }

  void anglesFromCoordinates(float x,float y,float z)
  {
	  /*
	http://www.learnaboutrobots.com/inverseKinematics.htm
	*/
	// Radius to the center of the tool.
	radiusTool = math.sqrt(pow(x, 2) + pow(y, 2))
	// Radius to joint3.
	radius = radiusTool - distanceTool
	baseAngle = math.atan2(y, x)
	// X coordinate of joint3.
	jointX = radius * math.cos(baseAngle)
	// Y coordinate of joint3.
	jointY = radius * math.sin(baseAngle)
	actualZ = z - heightFromGround
	// Imaginary segment connecting joint1 with joint2, squared.
	hypotenuseSquared = pow(actualZ, 2) + pow(radius, 2)
	hypotenuse = math.sqrt(hypotenuseSquared)

	q1 = math.atan2(actualZ, radius)
	q2 = math.acos((lengthRearSquared - lengthFrontSquared + hypotenuseSquared) / (2.0 * lengthRearArm * hypotenuse))
	rearAngle = piHalf - (q1 + q2)
	frontAngle = piHalf - (math.acos((lengthRearSquared + lengthFrontSquared - hypotenuseSquared) / (2.0 * lengthRearArm * lengthFrontArm)) - rearAngle)

	//return (baseAngle, rearAngle, frontAngle)
  }
// Dimentions in mm
#define lengthRearArm		SCARA_LINKAGE_1
#define lengthFrontArm		SCARA_LINKAGE_2
// Horizontal distance from Joint3 to the center of the tool mounted on the end effector.
#define distanceTool		50.9
// Joint1 height.
#define heightFromGround	80.0

#define lengthRearSquared	sq(lengthRearArm)
#define lengthFrontSquared  sq(lengthFrontArm)

#define armSquaredConst		(sq(lengthRearArm) + sq(lengthFrontArm))
#define armDoubledConst		(2.0 * lengthRearArm * lengthFrontArm)
#define radiansToDegrees	(180.0 / pi)
#define degreesToRadians	(pi / 180.0)

#define piHalf				pi / 2.0
#define piTwo				pi * 2.0
#define piThreeFourths		pi * 3.0 / 4.0
void inverse_kinematics(const float raw[XYZ])
{	
    static float C2, S2, SK1, SK2, THETA, PSI,R,r,jointX,jointY;

    float actualZ = raw[Z_AXIS] - SCARA_OFFSET_Z; 

	baseAngle = ATAN2(sy,sx);
    radiusTool = sqrt(sq(sx) + sq(sy));  
	radius = radiusTool - SCARA_OFFSET_R;
	jointX = radius*cos(baseAngle);
	jointY = radius*sin(baseAngle);

    hypotenuseSquared = sq(actualZ) + sq(radius)
	hypotenuse = sqrt(hypotenuseSquared)
	q1 = ATAN2(actualZ, radius);
	q2 = ACOS((lengthRearSquared - lengthFrontSquared + hypotenuseSquared) / (2.0 * lengthRearArm * hypotenuse));

	rearAngle = piHalf - (q1 + q2)
	frontAngle = piHalf - (ACOS((lengthRearSquared + lengthFrontSquared - hypotenuseSquared) / (2.0 * lengthRearArm * lengthFrontArm)) - rearAngle)

}
  void forward_kinematics_SCARA(const float &a, const float &b) {

    float a_sin = sin(RADIANS(a)) * L1,
          a_cos = cos(RADIANS(a)) * L1,
          b_sin = sin(RADIANS(b)) * L2,
          b_cos = cos(RADIANS(b)) * L2;

    cartes[X_AXIS] = a_cos + b_cos + SCARA_OFFSET_X;  //theta
    cartes[Y_AXIS] = a_sin + b_sin + SCARA_OFFSET_Y;  //theta+phi

    /*
      SERIAL_ECHOPAIR("SCARA FK Angle a=", a);
      SERIAL_ECHOPAIR(" b=", b);
      SERIAL_ECHOPAIR(" a_sin=", a_sin);
      SERIAL_ECHOPAIR(" a_cos=", a_cos);
      SERIAL_ECHOPAIR(" b_sin=", b_sin);
      SERIAL_ECHOLNPAIR(" b_cos=", b_cos);
      SERIAL_ECHOPAIR(" cartes[X_AXIS]=", cartes[X_AXIS]);
      SERIAL_ECHOLNPAIR(" cartes[Y_AXIS]=", cartes[Y_AXIS]);
    //*/
  }
  
	def coordinatesFromAngles(self, baseAngle, rearArmAngle, frontArmAngle):
		radius = lengthRearArm * math.cos(rearArmAngle) + lengthFrontArm * math.cos(frontArmAngle) + distanceTool
		x = radius * math.cos(baseAngle)
		y = radius * math.sin(baseAngle)
		z = heightFromGround - lengthFrontArm * math.sin(frontArmAngle) + lengthRearArm * math.sin(rearArmAngle)

		return (x, y, z)