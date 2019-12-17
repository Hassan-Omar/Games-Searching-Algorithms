using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Text; 

public class NodeNetworkCreator : MonoBehaviour {

    // var as an array of string for reading files
    private string[] maze;

	public IDictionary<Vector3, bool> walkablePositions = new Dictionary<Vector3, bool>();
	public IDictionary<Vector3, GameObject> nodeReference = new Dictionary<Vector3, GameObject>();
	public Dictionary<Vector3, string> obstacles = new Dictionary<Vector3, string>();
    public IList<Vector3> goals = new List<Vector3>(); 
	// Use this for initialization
	void Start () {
        readFile();

        InitializeNodeNetwork ();
	}
	

	void InitializeNodeNetwork(){
		
		var node = GameObject.Find ("Node");
        var obstacle = GameObject.Find("Obstacle");
        var agent = GameObject.Find("Agent");

        /* this part of code to color wakable node but with a diffrent weight to get the best path cost but now it is off 
         * as it doesn't make sense that each node have a different speed on the other node 
         */ 
        //Sprite wakableTile = Resources.Load<Sprite> ("obstacle_slow 1");
        //Sprite verySlowTile = Resources.Load<Sprite> ("obstacle_veryslow 1");

        // hold the goal object to add it as wakable object 
        GameObject goal = GameObject.Find("Goal");

        for (int i = 0; i < calculateLinesNumber(); i++) {

            // var to hold the current line value as char array 
            char[] temp = maze[i].ToCharArray();

            for (int j = 0; j < temp.Length; j++) {
				Vector3 curr_Position = new Vector3 (i, 0, j);

                if (temp[j] == '%')
                {
                    if (!obstacles.ContainsKey(curr_Position))
                    {
                        obstacles.Add(curr_Position, "barrier");
                    }

                }
                else if (temp[j] == '.')
                {
                    //goal.transform.localPosition = new Vector3(i, 0, j);
                    Instantiate(goal,curr_Position,goal.transform.rotation);
                    goals.Add(curr_Position);
                }
                else if (temp[j] == 'P')
                {
                    // move the agent to start point 
                    agent.transform.localPosition = curr_Position;
                    Agent.agentStartPosition = curr_Position;
                }
                GameObject copy;
				string obstacleType = null;

				if (obstacles.TryGetValue (curr_Position, out obstacleType))
                {
					copy = Instantiate (obstacle);
					copy.transform.position = curr_Position;
					
					 if(obstacleType=="barrier")
						walkablePositions.Add (new KeyValuePair<Vector3, bool> (curr_Position, false));
				}
				else {
					copy = Instantiate (node);
					copy.transform.position = curr_Position;
					walkablePositions.Add (new KeyValuePair<Vector3, bool> (curr_Position, true));
				}

				nodeReference.Add (curr_Position, copy);
			}
		}
        
        //walkablePositions[goal.transform.localPosition] = true;
        //nodeReference[goal.transform.localPosition] = goal;
    }


    // method to read the text file content then initialize maze array 
    /*
     * same java code open stream on file and iterate while end of file 
     */
    private void readFile()
    {

        // counter of lines 
        int i = 0;
        //string ma = Application.dataPath +  "/maze.txt";
        var fileStream = new FileStream(Application.dataPath + "/maze.txt", FileMode.Open, FileAccess.Read);
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

    // method to calculate the line numbers in a text file 
    /*
     *  I used this method to count the number of line in the file to know the height of the map 
     *  as when i use maze.lenght this returns a larg number and i can't understand where is the problem
     */
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
