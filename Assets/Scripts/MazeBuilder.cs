using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Text;
using UnityEngine;

public class MazeBuilder : MonoBehaviour
{
    // declare  all objs that i  will need it in my game  
    public GameObject wallUnit;
    public GameObject terrainUnit;
    public GameObject agent;
    public GameObject Target;
    // declare all parent to my objs 
    public GameObject wallUnits_Parent;
    public GameObject terrainUnits_Parent;
    public GameObject Targets_Parent;

    // vars for reading files
    private string [] maze;  

    // Start is called before the first frame update
    void Start()
    {
        readFile();
        
        /* As all Objs will be @ defined place on y axis so i will need only to 2 vars 
         * i is var will act as x axis 
         * j is var will act as z axis
         * and 
         * terrain-unit ------> @ y=0 
         * wall-unit    ------> @ y=0.65
         * agent        ------> @ y=0.16
         * target       ------> @ y=0.6
         */

        for (int i = 0; i < maze.Length; i++)
        {
             // var to hold the current line value as char array 
            char[] temp = maze[i].ToCharArray(); 
            // inner loop to iterates on the current line 
            for (int j = 0; j < temp.Length; j++)
            {
                // now i will make the terrain at y=0 then i will check if i have wall-unit or agent or target at this place 

                Instantiate(terrainUnit,new Vector3(i,0,j),Quaternion.identity, terrainUnits_Parent.transform);

                // start check 
                if (temp[j] == '%')
                    Instantiate(wallUnit, new Vector3(i, 0.65f, j), Quaternion.identity, wallUnits_Parent.transform);
                else if (temp[j] == '.')
                    Instantiate(Target, new Vector3(i, 0.6f, j), Quaternion.identity, Targets_Parent.transform);
                else if (temp[j] == 'P')
                    Instantiate(agent, new Vector3(i, 0.16f, j), Quaternion.identity);

            }

        }
        
        
       
    }


    private void readFile()
    {

        // counter of lines 
        int i = 0;
        //string ma = Application.dataPath +  "/maze.txt";
        var fileStream = new FileStream(Application.dataPath+"/maze.txt", FileMode.Open, FileAccess.Read);
        maze = new string[calculateLinesNumber()];

        using (var streamReader = new StreamReader(fileStream, Encoding.UTF8))
        {
            string line;
            while ((line = streamReader.ReadLine()) != null)
            {
                // process the line
                maze[i] = line;
                i++;
            }
        }
         ;

    }

    private int calculateLinesNumber()
    { 
        int i = 0; 
        var fileStream = new FileStream(Application.dataPath + "/maze.txt", FileMode.Open, FileAccess.Read);
        using (var streamReader = new StreamReader(fileStream, Encoding.UTF8))
        {
            string line;
            while ((line = streamReader.ReadLine()) != null)
            { 
                i++;
            }
        }
         ;
        return i; 
    }
}
