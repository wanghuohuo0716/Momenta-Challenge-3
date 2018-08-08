Header header;
typedef struct
{
    double x;
    double y;
}Ball;

typedef struct
{
    double x;
    double y;
}Door;

typedef struct
{
    double x;
    double y;
}Enemy;

typedef struct
{
    Ball Ball;
    Door door;
    Enemy enemy;
}global_pos;