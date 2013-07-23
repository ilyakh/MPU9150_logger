import matplotlib.pyplot as pyplot
import pandas
import sys
import time
from pandas import DataFrame

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

    timedelta_average = ( float(sum( delta_t )) / len( delta_t ) )
    
    timedeltas_above_double_average = \
        [ i for i in delta_t if (i > ( 2.0 * timedelta_average )) ]

    print "Maximal timedelta", max( delta_t )
    print "Average timedelta", timedelta_average
    
    
    timedeltas_above_double_average_percent = \
        float( len( timedeltas_above_double_average ) ) / len( delta_t ) * 100

    print "Timedeltas above double average",
    print len(timedeltas_above_double_average), 
    print timedeltas_above_double_average_percent

    last_timestamp = timestamps[-1]

    print "Last timestamp", timestamps[-1]
    print "Maximal timestamp", max( timestamps )
    print "Average frequency", float( len( timestamps ) ) / ( float( last_timestamp ) / 1000 ) 


    delta_t = DataFrame( delta_t )
    delta_t.plot()
    pyplot.show()

    print "Timedelta standard deviation", float( delta_t.std() )

    font = {
        'family': 'Consolas',
        'weight': 'x-small',
        'size': 11.0,
        'stretch': 0
    }

    
    
    pyplot.rc( 'font', **font )
    pyplot.show( block=True )    
