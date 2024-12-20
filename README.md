# hdc_sw

Setup the enviroment:

```bash
source env_setup.sh
```


## To Run the HDC C-Model

### DATA CONVERTER

First we run `example.py` to train and save the model and a few test vectors.
Running `example_load.py` will load the saved model and test it on the saved test vectors. The model weights are also put into a file `weightspy.txt`. 

The correctness of the exported weights is checked with `load_weights.c`. This will load from the saved weights and output as text to `weightsc.txt` which should be identical to `weightspy.txt`

These steps are performed with:
``` bash
make python_model

```
REQURIED gcc version > 9.0

#### Expected Std-Output from Python Model Training and Weight Export

```bash
Saving dataset samples...
Training...
Validating...
ACCURACY tensor(0.9958)
ACCURACY TEST tensor(0.9652)
TIME 5.461008548736572
Saving model attributes...
python onlinehd/example_load.py
Loading...
Loading model attributes...
Validating...
acc = 1.000000
./load_weights
PASS: Exported Weights and Python Model are identical
```
### Compile C Model

The onlineHD model has been converted to C with a flat implementation suitable for HLS. Currently only inference is supported.

The testbench loads the .bin weight exporeted from the previous step, and initiatilizes the model. 

To compile the C-Model and Test:
```bash 
make test
./test
```

#### Expected Std-Output from C Model
```bash
$ ./test
rows images: 100 
model size: 160000 bytes (40000 floats)
Image 1:
                            
                            
                            
                            
                            
                            
                            
        #############       
       ##############       
       ##############       
       ##############       
       ######## ####        
        ####    ####        
                ####        
               #####        
              #####         
              #####         
             #####          
            #####           
            ####            
           #####            
          #####             
         #####              
         #####              
        #####               
        ####                
        ####                
                            

Image 2:
                            
                            
    #                ##     
                    ###     
                    ###     
                  ####      
                 ####       
                ####        
               ####         
              #####         
            #####           
           #####            
          ######  ###       
         #####  ######      
        ##### #########     
        ###############     
       ###############      
      ################      
      ###############       
      #############         
      ##########            
        #####               
                            
                            
                            
                            
                            
                            

Image 3:
                            
                            
                            
                            
                            
                            
                            
                            
   ###############          
  ##################        
  ###################       
  ###        ########       
                ######      
                  ####      
                  ####      
                  ####      
                  ####      
                  ####      
                 ####       
                 ####       
                 ####       
                ####        
                ####        
               ####         
               ####         
              ####          
              ####          
             ####           

Image 4:
                            
                            
                            
                            
                            
              ###           
             ####           
           ########         
           #########        
          ##### ####        
          ####   ###        
          ###   ####        
          ###  ####         
          #########         
           #######          
            #####           
             ####           
            #####           
            ######          
            ######          
           ### ###          
           #######          
           ######           
           ######           
           #####            
                            
                            
                            

Image 5:
                            
                            
                            
                            
                            
              ####          
              ####          
              ####          
              ####          
             ####           
             ####           
             ####           
             ####           
            ####            
            ####            
            ####            
           #####            
           #####            
           ####             
           ####             
           ####             
           ####             
          #####             
          ####              
           ###              
                            
                            
                            

y[0] is 7 
y[1] is 6 
y[2] is 7 
y[3] is 8 
y[4] is 1 
predictions[0] is 7 
predictions[1] is 6 
predictions[2] is 7 
predictions[3] is 8 
predictions[4] is 1 
Accuracy of the model: 95.00%

```