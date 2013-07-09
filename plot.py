import matplotlib.pyplot as pyplot
import pandas
import sys
import time

if __name__ == "__main__":

    number = sys.argv[1]

    RAW_ONLY = True
    DMP_ONLY = False

    try:

        data = pandas.io.parsers.read_csv( 
            "/Volumes/NO NAME/LOG{number}.TXT".format( number=number ),
            index_col=False,
            names=['raw_x', 'raw_y', 'raw_z'], 
                  # 'dmp_x', 'dmp_y', 'dmp_z' ],
            na_values=['nan'],
        )

        if RAW_ONLY:
            data = data[['raw_x', 'raw_y', 'raw_z']].copy()
        
        elif DMP_ONLY:
            data = data[['dmp_x', 'dmp_y', 'dmp_z']].copy()
            

    except IOError:
        sys.exit("file was not found")
    
    

    print "Number of readings:", len( data ) 
    
    p = data.plot()


    font = {
        'family': 'Consolas',
        'weight': 'x-small',
        'size': 11.0,
        'stretch': 0
    }

    
    
    pyplot.rc( 'font', **font )
    pyplot.show( block=True )

    print data.head()
    
    
