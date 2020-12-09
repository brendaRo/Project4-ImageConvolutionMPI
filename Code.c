#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION
#define STB_IMAGE_RESIZE_IMPLEMENTATION
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <omp.h>
#include <time.h>
#include "stb_image.h"
#include "stb_image_write.h"
#include "stb_image_resize.h"
#include "mpi.h"


int readmatrix(size_t rows, size_t cols, double (*a)[cols], const char* filename)
{
    size_t v, q;
    FILE *pf;
    pf = fopen (filename, "r");
    if (pf == NULL)
        return 0;

    for( v = 0; v < rows; v++)
    {
        for( q = 0; q < cols; q++)
            fscanf(pf, "%lf", &a[v][q]);
    }

    fclose (pf);
    return 1;
}


int main(int argc, char *argv[]){


	char *imgpath = argv[1];
	char *filterpath = argv[2];
	char *newimgpath = argv[3];
	char *filtsize = argv[74;


            int i, j, k, m, n, h, w, b,r, colorc = 3, grayc = 1;
            FILE *fil;
            double filter[atoi(filtsize)][atoi(filtsize)], t0, t1;
            readmatrix(atoi(filtsize), atoi(filtsize), filter, filterpath); //creation of the filter matrix


            uint8_t* image = stbi_load(imgpath, &w, &h, &b, colorc); //image vector
            int img_size = w * h * colorc, wr = w-(atoi(filtsize)-1), hr = h-(atoi(filtsize)-1);
            int gray_img_size, red_img_s = hr * wr * grayc; //size of gray images
            unsigned char *gray_img = malloc(gray_img_size*10), *new_img_gray = malloc(gray_img_size*10), *reduced = malloc(red_img_s*10); //gray imgs + reduced


            int tasks, IDtask, workers, work, source, dest, par, filas, av, offset = atoi(filtsize);
            MPI_Status status;

//transforming of RGB image to gray scale image

            for(n = 0; n <=  gray_img_size; n++) {
                unsigned char *pg = gray_img;
                unsigned char *p = image;

                *(pg + n) = (uint8_t)((*(p+ n*colorc)*0.21 + *(p + n*colorc + 1)*0.72 + *(p + n*colorc + 2)*0.07)); //luminosity
            }

            *gray_img = pg;

        //MPI initialization
        MPI_Init(&argc,&argv);
        MPI_Comm_rank(MPI_COMM_WORLD,&IDtask);
        MPI_Comm_size(MPI_COMM_WORLD,&tasks);
        workers = tasks-1;

    for (r = 0, r < 10, r++){


//Master
        if (taskid == MASTER)
   {
        filtro = filter;
        gray_img = malloc(gray_img_size);
        new_img_gray = malloc(gray_img_size);
        av = h/numworkers;
        offset = 0;

        work = FROM_MASTER; //type of worker that sends data

        t0 = MPI_Wtime(); //start time
      for (dest = 1; dest <= numworkers; dest++)
      {
         filas = (dest <= extra)?av+1 : av;
         MPI_Send(&offset, 1, MPI_INT, dest, work, MPI_COMM_WORLD);
         MPI_Send(&filas, 1, MPI_INT, dest, work, MPI_COMM_WORLD);
         MPI_Send(&w, 1, MPI_INT, dest, work, MPI_COMM_WORLD);
         MPI_Send(gray_img+(offset*w), (filas+offs)*w, MPI_UNSIGNED_CHAR, dest, work,MPI_COMM_WORLD);
         MPI_Send(filtro, atoi(filtsize)*atoi(filtsize), MPI_DOUBLE, dest, work, MPI_COMM_WORLD);
         offset = offset + filas;
      }

      work = FROM_WORKER; // type of worker that receives
      for (i=1; i<=numworkers; i++)
      {
         source = i;
         MPI_Recv(&offset, 1, MPI_INT, source, work, MPI_COMM_WORLD, &status); //variable, size, type of variable, from who, type
         MPI_Recv(&filas, 1, MPI_INT, source, work, MPI_COMM_WORLD, &status);
         MPI_Recv(new_img_gray+(offset*w), filas*w, MPI_DOUBLE, source, work, MPI_COMM_WORLD, &status);
         }

       t1 = MPI_Wtime(); //end time


    //resize and write image
            stbir_resize_uint8(new_img_gray , w , h , 0, reduced, wr, hr, 0, 1);
            stbi_write_jpg(newimgpath, wr, hr, grayc, reduced, 100);

    //free memories
            stbi_image_free(image);
            free(gray_img);
            free(new_img_gray);
            free(reduced);
            free(filter);


    //print time
            printf("Total_convolution_time:%fms\n", g, (double)(t1-t0)*1000);

   } //  end master

//Workers
    if (IDtask > MASTER) {

            work = FROM_MASTER; //type of the worker that we receive info

            //receive from master the values
            MPI_Recv(&w, 1, MPI_INT, MASTER, work, MPI_COMM_WORLD, &status);
            MPI_Recv(&filas, 1, MPI_INT, MASTER, work, MPI_COMM_WORLD, &status);
            MPI_Recv(&offset, 1, MPI_INT, MASTER, work, MPI_COMM_WORLD, &status);

            //Memories
            filtro = malloc(atoi(filtsize)*atoi(filtsize)*sizeof(double *));
            gray_img_size = (filas + offset) * w;
            gray_img = malloc(gray_img_size);
            new_img_gray = calloc(filas, w)

            //receive from master
            MPI_Recv(gray_img, (filas+offs)*w, MPI_UNSIGNED_CHAR, MASTER, work, MPI_COMM_WORLD, &status);
            MPI_Recv(filtro, atoi(filtsize)*atoi(filtsize), MPI_DOUBLE, MASTER, work, MPI_COMM_WORLD, &status);

            //convolution of the gray image and the filter
            for (k=0; k <= filas; k++){
                    for (l = 0, l < w, l++){
                            unsigned char *pg = new_img_gray;
                            unsigned char *p = gray_img;
                            int i, j;
                            if (l+atoi(filtsize) <= w){
                                    unsigned char sum_loc = 0;
                                    for (i = atoi(filtsize)-1; i >= 0; i--){ //convolution for digital images

                                        for (j = atoi(filtsize)-1; j >= 0; j--){

                                            sum_loc += filter[i][j] * *(p + k +(atoi(filtsize)-(j+1)) + (w*(atoi(filtsize)-(i+1))));
                                        }
                                    }

                //save convolution result into the new image
                *(pg + k) = (uint8_t)sum_loc;
                            }
                    }
            }
            new_img_gray = pg;

            work = FROM_WORKERS; //type of worker that sends info
            MPI_Send(&offset, 1, MPI_INT, MASTER, work, MPI_COMM_WORLD);
            MPI_Send(&filas, 1, MPI_INT, MASTER, work, MPI_COMM_WORLD);
            MPI_Send(new_img_gray, filas*w, MPI_UNSIGNED_CHAR, MASTER, work, MPI_COMM_WORLD);
} //if workers

} //executions

MPI_Finalize();
}
