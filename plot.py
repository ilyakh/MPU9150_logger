import matplotlib.pyplot as pyplot
import pandas
import sys
import time

if __name__ == "__main__":

    number = sys.argv[1]

    try:

        data = pandas.io.parsers.read_csv( 
            "/Volumes/TUSENFRYD2/LOG{number}.TXT".format( number=number ),
            index_col=False,
            names=['time',
                   'accel_x', 'accel_y', 'accel_z', 
                   'gyro_x', 'gyro_y', 'gyro_z' ],
            na_values=['nan'],
        )


            

    except IOError:
        sys.exit("file was not found")
    
    

    print "Number of readings:", len( data ) 
    
    p = data.plot()

    timestamps = list( data['time'].values )

    delta_t = []
    for i in range( len(timestamps) -1 ):
        delta_t.append(
            timestamps[i+1] - timestamps[i]
        )

    print delta_t[0:50]

    print "Maximal timedelta", max( delta_t )
    print "Average timedelta", ( sum( delta_t) / len( delta_t ) )


    font = {
        'family': 'Consolas',
        'weight': 'x-small',
        'size': 11.0,
        'stretch': 0
    }

    
    
    pyplot.rc( 'font', **font )
    pyplot.show( block=True )    
