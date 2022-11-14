struct tracking_value {
    double position; // Y-position of limelight
    double perc; // Rotational speed percentage
    double angle; // Angle to aim at
};


const tracking_value interpolationValues[] = {
    {
        0, // Y-position (percentage) (furthest possible up)
        0.8, // Power (percentage)
        0.5 // Angle (percentage)
    },
    {
        0.15,
        0.85,
        0.7
    },
    {
        0.25,
        0.75,
        0.8
    },
    {
        0.35,
        0.86,
        0.8
    },
    {
        0.5,
        0.8,
        0.9
    },
    {
        0.6,
        1.3,
        0.7
    },
    {
        1, // (furthest possible down)
        1.3,
        0.7
    },
    { // Watchdog, Python style.
        2,
        2,
        2
    }
};
