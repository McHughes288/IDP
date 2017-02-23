#include <iostream>
#include <string>
#include <cmath>
#include <vector>

#include "global.h"
#include "matrix.h"

//#define DEBUG

using namespace std;

string string_matrix[18][15] = {
{"x",   "x",   "x",   "x",   "x",   "x",   "x",   "x",   "x",   "x",   "x",   "x",   "x",   "x",   "x"},
{"x",   "x",   "x",   "x",   "x",   "x",   "T6",  "0",   "T5",  "0",   "T7",  "x",   "D1",  "x",   "x"},
{"x",   "x",   "x",   "x",   "0",   "x",   "0",   "x",   "0",   "x",   "0",   "x",   "0",   "x",   "x"},
{"x",   "x",   "x6",  "0",   "J10", "0",   "J11", "0",   "j12", "0",   "j13", "0",   "j16", "0",   "c2"},
{"x",   "x",   "x",   "x",   "0",   "x",   "0",   "x",   "0",   "x",   "0",   "x",   "0",   "x",   "x"},
{"x",   "x",   "x",   "x",   "0",   "x",   "T8",  "0",   "J14", "0",   "T9",  "x",   "D2",  "x",   "x"},  
{"x",   "x",   "x",   "x",   "0",   "x",   "x",   "x",   "0",   "x",   "x",   "x",   "x",   "x",   "x"},
{"x",   "x",   "x",   "x",   "0",   "x",   "x",   "x",   "D32", "x",   "x",   "x",   "x",   "x",   "x"},
{"x",   "x",   "x",   "x",   "0",   "x",   "x",   "x",   "x",   "x",   "x",   "x",   "x",   "x",   "x"},
{"x",   "x",   "x",   "x",   "0",   "x",   "x",   "x",   "D31", "x",   "x",   "x",   "x",   "x",   "x"},
{"x",   "x",   "x",   "x",   "0",   "x",   "x",   "x",   "0",   "x",   "x",   "x",   "x",   "x",   "x"},
{"x",   "x",   "x",   "x",   "0",   "x",   "x5",  "0",   "J3",  "x6",  "T2",  "x",   "x",   "x",   "x"},
{"x",   "x",   "T5",  "0",   "J7",  "0",   "T4",  "x4",  "J17", "0",   "J2",  "x3",  "x",   "x",   "x"},
{"x",   "x",   "0",   "x",   "0",   "x",   "0",   "x",   "0",   "x",   "0",   "x",   "x",   "x",   "x"},
{"x2",  "0",   "J8",  "0",   "J6",  "0",   "J5",  "0",   "J4",  "0",   "J1",  "0",   "C1",  "x",   "x"},
{"x",   "x",   "0",   "x",   "0",   "x",   "0",   "x",   "0",   "x",   "0",   "x",   "x",   "x",   "x"},
{"x",   "x",   "T3",  "0",   "J9",  "0",   "T2",  "x",   "P2",  "x",   "P1",  "x",   "x",   "x",   "x"},
{"x",   "x",   "x",   "x",   "x1",  "x",   "x",   "x",   "x",   "x",   "x",   "x",   "x",   "x",   "x"},
};

int int_matrix[18][15] = {
{-1,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
{-1,  -1, -1, -1, -1, -1,  0,  0,  0,  0,  0, -1,  0, -1, -1},
{-1,  -1, -1, -1,  0, -1,  0, -1,  0, -1,  0, -1,  0, -1, -1},
{-1,  -1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0},
{-1,  -1, -1, -1,  0, -1,  0, -1,  0, -1,  0, -1,  0, -1, -1},
{-1,  -1, -1, -1,  0, -1,  0,  0,  0,  0,  0, -1,  0, -1, -1},
{-1,  -1, -1, -1,  0, -1, -1, -1,  0, -1, -1, -1, -1, -1, -1},
{-1,  -1, -1, -1,  0, -1, -1, -1,  0, -1, -1, -1, -1, -1, -1},
{-1,  -1, -1, -1,  0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
{-1,  -1, -1, -1,  0, -1, -1, -1,  0, -1, -1, -1, -1, -1, -1},
{-1,  -1, -1, -1,  0, -1, -1, -1,  0, -1, -1, -1, -1, -1, -1},
{-1,  -1, -1, -1,  0, -1, -2,  0,  0,  -2, 0, -1, -1, -1, -1},
{-1,  -1,  0,  0,  0,  0,  0, -2,  0, -0,  0, -2, -1, -1, -1},
{-1,  -1,  0, -1,  0, -1,  0, -1,  0, -1,  0, -1, -1, -1, -1},
{-2,   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, -1, -1},
{-1,  -1,  0, -1,  0, -1,  0, -1,  0, -1,  0, -1, -1, -1, -1},
{-1,  -1,  0,  0,  0,  0,  0, -1,  0, -1,  0, -1, -1, -1, -1},
{-1,  -1, -1,-1   -2, -1, -1, -1, -1,  -1, -1, -1, -1, -1, -1},
};

int coded_matrix[18][15] = {
{-1, -1, -1, -1, -1, -1, -1,  -1, -2, -1, -1, -1, -1,  -1, -1},
{-1, -1, -1, -1, -2, -1,  2,   0,  1,  0,  2, -1,  5   -1, -1},
{-1, -1, -1, -1,  0, -1,  0,  -1,  0, -1,  0, -1,  0,  -1, -1},
{-1, -1,  0,  0,  1,  0,  1,   0,  1,  0,  1,  0,  1,   0,  11},
{-1, -1, -1, -1,  0, -1,  0,  -1,  0, -1,  0, -1,  0,  -1, -1},
{-1, -1, -1, -1,  0, -1,  2,   0,  1,  0,  2, -1,  6,  -1, -1},
{-1, -1, -1, -1,  0, -1, -1,  -1,  0, -1, -1, -1, -1,  -1, -1},
{-1, -1, -1, -1,  0, -1, -1,  -1,  7, -1, -1, -1, -1,  -1, -1},
{-1, -1, -1, -1,  0, -1, -1,  -1, -1, -1, -1, -1, -1,  -1, -1},
{-1, -1, -1, -1,  0, -1, -1,  -1,  7, -1, -1, -1, -1,  -1, -1},
{-1, -1, -1, -1,  0, -1, -1,  -1,  0, -1, -1, -1, -1,  -1, -1},
{-1, -1, -1, -1,  0, -1, -2,   0,  1,  -2,  2, -1, -1,  -1, -1},
{-1, -1,  2,  0,  0,  0,  2,  -2,  1,  0,  0, -2, -1,  -1, -1},
{-1, -1,  0, -1,  0, -1,  0,  -1,  0, -1,  0, -1, -1,  -1, -1},
{-2,  0,  1,  0,  1,  0,  1,   0,  1,  0,  1,  0,  10, -1, -1},
{-1, -1,  0, -1,  0, -1,  0,  -1,  0, -1,  0, -1, -1,  -1, -1},
{-1, -1,  2,  0,  0,  0,  2,  -1,  9, -1,  8, -1, -1,  -1, -1},
{-1, -1, -1, -1, -2, -1, -1,  -1, -1, -1, -1, -1, -1,  -1, -1},
};

void shortest_path(int start_x, int start_y, int f_x, int f_y);
vector<int> path_x,path_y, path_n; // path[x][y][#]

vector<int> final_x,final_y,final_n;

int main()
{	
	cout << "start = " << int_matrix[2][4] << "  finish = " << int_matrix[5][4] << endl;
	shortest_path(2,4,5,4);
	
	cout << "Path: " << endl;
	for(int i = 0; i < (int)final_x.size(); i++)
	{
		cout  << "(" << final_x[i] << "," << final_y[i] << "," << final_n[i] << ")" << endl;
	}
	
	return 0;
}

void shortest_path(int start_x, int start_y, int f_x, int f_y)
{
	int k = 0;
	vector<int> temp_x, temp_y, temp_n, near_x, near_y, near_n;
	temp_y.push_back(f_y);
	temp_x.push_back(f_x);

	
	#ifdef DEBUG
	int counter = 1;
	#endif
	
	//int copy_matrix[18][15];
	
	if(start_x < 0 || start_y < 0 || f_x < 0 || f_y < 0 )
	{
		cerr << "Invalid start or finish coordinate" << endl;
		return;
	}
	
	else if(start_x >15 || start_y > 18 || f_x >15 || f_y > 18 )
	{
		cerr << "Invalid start or finish coordinate" << endl;
		return;
	}
	
	else if(int_matrix[start_x][start_y] == -1 || int_matrix[f_x][f_y] == -1)
	{
		cerr << "Coordinate entered is a wall" << endl;
		return;
	}
	
	
	if(!path_x.empty() || !path_y.empty() || !path_n.empty())
	{
		cout << "Erased vectors" << endl;
		path_x.erase(path_x.begin(), path_x.end());
		path_y.erase(path_y.begin(), path_y.end());
		path_n.erase(path_n.begin(), path_n.end());
	}
	
	/*for(int i = 0; i < 18; i++)
	{
		for(int j = 0; j < 15; j++)
		{
			copy_matrix[i][j] = int_matrix[i][j];
		}
	}*/
	
	path_x.push_back(start_x);
	path_y.push_back(start_y);
	path_n.push_back(0);
	k++;
	
	while(1)
	{
		#ifdef DEBUG
		cout << "Entered while loop" << endl;
		cout << "Size of path vector = " << path_x.size() << endl;
		#endif
		int size = path_x.size();
		
		for(int i = 0; i < size; i++) //add adjacent cells to the list
		{
			path_x.push_back(path_x[i] + 1);
			path_y.push_back(path_y[i]);
			path_n.push_back(k);
			
			path_x.push_back(path_x[i] - 1);
			path_y.push_back(path_y[i]);
			path_n.push_back(k);
			
			path_x.push_back(path_x[i]);
			path_y.push_back(path_y[i] + 1);
			path_n.push_back(k);
			
			path_x.push_back(path_x[i]);
			path_y.push_back(path_y[i] - 1);
			path_n.push_back(k);
			
			#ifdef DEBUG
			cout << "Added 4 new elements" << endl;
			#endif
			
		}
		k++;
		
		#ifdef DEBUG
		cout << "Created adjacent points" << endl;
		for(int i= 0; i<(int)path_x.size(); i++)
		{
			cout << endl;
			cout  << "(" << path_x[i] << "," << path_y[i] << "," << path_n[i] << ")" << endl;
		}
		#endif
		
		for(int i = 0; i < (int)path_x.size(); i++)
		{
			
			
			if(int_matrix[path_x[i]][path_y[i]] == -1) //remove any walls from the list
			{
				#ifdef DEBUG 
				cout << "Deleting: " << "(" << path_x[i] << "," << path_y[i] << "," << path_n[i] << ")" << endl;
				#endif
				
				path_x.erase(path_x.begin() + i ); //-1 to get the ith element
				path_y.erase(path_y.begin() + i );// -1 to get the ith element
				path_n.erase(path_n.begin() + i );//-1 to get the ith element
				i = 0; // to avoid elements being transfered to the previous element and thus not being checked
			}
			
		}
		
		#ifdef DEBUG
		cout << "Removed walls, current length of vector = " << path_x.size() << endl;
		#endif
		
		for(int i = 0; i < (int)path_x.size(); i++) //remove duplicate element with the highest number
		{
			for(int j = 0; j < (int)path_x.size(); j++)
			{
				if(path_x[i] == path_x[j] && path_y[i] == path_y[j] && i != j)
				{
					if(path_n[i] < path_n[j])
					{
						#ifdef DEBUG 
						cout << "Deleting: " << "(" << path_x[j] << "," << path_y[j] << "," << path_n[j] << ")" << endl;
						#endif
						path_x.erase(path_x.begin() + j );
						path_y.erase(path_y.begin() + j );
						path_n.erase(path_n.begin() + j );
					}
					else
					{
						#ifdef DEBUG 
						cout << "Deleting: " << "(" << path_x[i] << "," << path_y[i] << "," << path_n[i] << ")" << endl;
						#endif
						path_x.erase(path_x.begin() + i );
						path_y.erase(path_y.begin() + i );
						path_n.erase(path_n.begin() + i );
					}
					i = j = 0; //to avoid elements not being checked
				}
			}
		}
		
		
		#ifdef DEBUG
		cout << "Removed duplicates current length of vector = " << path_x.size()  << endl;
		
		counter++;
		
		if(counter == -1) //ie don't break
		{
			break;
		}
		#endif
		
		#ifdef DEBUG
		cout << "Remaining points: " << endl;
		for(int i= 0; i<(int)path_x.size(); i++)
		{
			cout << endl;
			cout  << "(" << path_x[i] << "," << path_y[i] << "," << path_n[i] << ")" << endl;
		}
		#endif
		
		
		for(int i = 0; i < (int)path_x.size(); i++)
		{
			if(path_x[i] == f_x && path_y[i] == f_y)
			{
				cout << "path found with " << path_n[i] << " steps" << endl;
				temp_n.push_back(path_n[i]);
				int counter = 0;
				int steps = path_n[i];
				
				for(int j = 0; j <= path_n[i];j++)
				{
					counter++;
					
					near_x.erase(near_x.begin(),near_x.end());
					near_y.erase(near_y.begin(),near_y.end());
					near_n.erase(near_n.begin(),near_n.end());
			
					near_x.push_back(temp_x[j] + 1);
					near_y.push_back(temp_y[j]);
					near_n.push_back(steps - counter);
			
					near_x.push_back(temp_x[j] - 1);
					near_y.push_back(temp_y[j]);
					near_n.push_back(steps - counter);
			
					near_x.push_back(temp_x[j]);
					near_y.push_back(temp_y[j] + 1);
					near_n.push_back(steps - counter);
		
					near_x.push_back(temp_x[j]);
					near_y.push_back(temp_y[j] - 1);
					near_n.push_back(steps - counter);
					
			
			
					for( int k = 0; k < (int)path_x.size(); k++)
					{
	
						for(int n = 0; n < (int)near_x.size(); n++)
						{
							if(near_x[n] == path_x[k] && near_y[n] == path_y[k] && near_n[n] == path_n[k])
							{
								temp_x.push_back(path_x[k]);
								temp_y.push_back(path_y[k]);
								temp_n.push_back(path_n[k]);
							}
						}
					}
				}
				
				final_x.erase(final_x.begin(),final_x.end());
				final_y.erase(final_y.begin(),final_y.end());
				final_n.erase(final_n.begin(),final_n.end());
				
				for(int k = 0; k < (int)temp_x.size(); k++)
				{
					final_x.insert(final_x.begin(), temp_x[k]);
					final_y.insert(final_y.begin(), temp_y[k]);
					final_n.insert(final_n.begin(), temp_n[k]);
				}
				return;
			}
		}
		
		
	}
	
	
}
