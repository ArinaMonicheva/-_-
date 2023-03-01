#define HAVE_STRUCT_TIMESPEC
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>

#define DT 0.05

double diffclock(clock_t clock1, clock_t clock2) {

    double diffticks = clock1 - clock2;
    double diffms = diffticks / (CLOCKS_PER_SEC / 1000);

    return diffms;
}

typedef struct
{
    double x, y;
} vector;

int bodies, timeSteps;
double* masses, GravConstant;
vector* positions, * velocities, * accelerations, * newPositions;
int thread_count;

vector addVectors(vector a, vector b)
{
    vector c = { a.x + b.x, a.y + b.y };

    return c;
}

vector scaleVector(double b, vector a)
{
    vector c = { b * a.x, b * a.y };

    return c;
}

vector subtractVectors(vector a, vector b)
{
    vector c = { a.x - b.x, a.y - b.y };

    return c;
}

double mod(vector a)
{
    return sqrt(a.x * a.x + a.y * a.y);
}

void initiateSystem(char* fileName)
{
    int i;
    FILE* fp = fopen(fileName, "r");

    fscanf(fp, "%lf%d%d", &GravConstant, &bodies, &timeSteps);

    masses = (double*)malloc(bodies * sizeof(double));
    positions = (vector*)malloc(bodies * sizeof(vector));
    newPositions = (vector*)malloc(bodies * sizeof(vector));
    velocities = (vector*)malloc(bodies * sizeof(vector));
    accelerations = (vector*)malloc(bodies * sizeof(vector));

    for (i = 0; i < bodies; i++)
    {
        fscanf(fp, "%lf", &masses[i]);
        fscanf(fp, "%lf%lf", &positions[i].x, &positions[i].y);
        fscanf(fp, "%lf%lf", &velocities[i].x, &velocities[i].y);
    }

    fclose(fp);
}

void resolveCollisions()
{
    int i, j;

    for (i = 0; i < bodies - 1; i++)
        for (j = i + 1; j < bodies; j++)
        {
            if (positions[i].x == positions[j].x && positions[i].y == positions[j].y)
            {
                vector temp = velocities[i];
                velocities[i] = velocities[j];
                velocities[j] = temp;
            }
        }
}

void computeAccelerations(int body)
{
    int j;
    accelerations[body].x = 0;
    accelerations[body].y = 0;
    for (j = 0; j < bodies; j++)
    {
        if (body != j)
        {
            accelerations[body] = addVectors(accelerations[body], scaleVector(GravConstant * masses[j] / pow(mod(subtractVectors(positions[body], positions[j])), 3), subtractVectors(positions[j], positions[body])));
        }
    }
}

void computeVelocities(int body)
{
    velocities[body] = addVectors(velocities[body], scaleVector(DT, accelerations[body]));
}

void computePositions(int body)
{
    newPositions[body] = addVectors(positions[body], scaleVector(DT, velocities[body]));
}

void* simulate(void* j)
{
    int i;
    int end = (int)j;
    for (i = end - bodies / thread_count; i < end; i++) {
        computeAccelerations(i);
        computePositions(i);
        computeVelocities(i);
    }

    return 0;
}

int main(int argC, char* argV[])
{
    int i, j;
    thread_count = strtol(argV[2], NULL, 10);
    pthread_t* thread_handles = malloc(thread_count * sizeof(pthread_t));

    if (argC != 3)
        printf("Usage : %s <file name containing system configuration data>", argV[0]);
    else
    {
        FILE* outputFile = fopen("S2.csv", "w");
        fprintf(outputFile, "Body   :     x              y           vx              vy   \n");
        initiateSystem(argV[1]);
        clock_t start = clock();
        for (i = 0; i < timeSteps; i++)
        {
            for (long thread = 0; thread < thread_count; thread++)
            {
                pthread_create(&thread_handles[thread], NULL, simulate, (void*)((bodies / thread_count) * (thread + 1))); //нада тут звездочка или нет
            }
            for (int thread = 0; thread < thread_count; thread++)
            {
                // join потому что потоки возвращаются/присоединяются обратно к главному потоку
                pthread_join(thread_handles[thread], NULL); //-второй аргумент нужен, чтобы получить возвращаемый результат функции
            }
            fprintf(outputFile, "Cycle %d\n", i);
            for (j = 0; j < bodies; j++) {
                fprintf(outputFile, "Body %d : %lf\t%lf\t%lf\t%lf\n", j + 1, newPositions[j].x, newPositions[j].y, velocities[j].x, velocities[j].y);
                positions[j] = newPositions[j];
            }
            resolveCollisions();
        }
        clock_t end = clock();
        printf("Time: %lf", diffclock(end, start));
        fclose(outputFile);
    }

    free(masses);
    free(positions);
    free(newPositions);
    free(velocities);
    free(accelerations);
    free(thread_handles);

    return 0;
}
