

/// so far best - it's stable somehow but it's fails to detect teh thrid one here and there still fine

#define MIN_CLUSTER_SIZE 2
#define MIN_CYLINDER_WIDTH 0.06  // HINT: must be > april_tag dimension (0.05 X 0.05 (M) -> 5cm) (The cylinders are bigger), 
#define MAX_CYLINDER_WIDTH 0.5  // Maximum cylinder diameter, half a meter )
#define CLUSTER_PROXIMITY_THRESHOLD 0.3  // 40cm
#define MAX_CYLINDER_DETECTION_RANGE 3 // 3 meters






/// Amd gonna do two experimetns



// THE REST make it the same now play with Cluster size, min cylinder width, and cluster promizity

// 1. smaller width & smaller proximty test the two

- here first set 
    #define MIN_CYLINDER_WIDTH 0.05
    #define CLUSTER_PROXIMITY_THRESHOLD 0.3  // 40cm

    case - 1: min_cylinder_width = 0.05
        observation:
            - it is so stable. (no 2 cylinder at all)
        :

    case - 2: cluster_proximity_threshold = 0.25 & 0.2


    case - 3: merge teh two tegother




// 2.  cluster size 1, woudl it be too little or two noisey - drop this one