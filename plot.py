import matplotlib.pyplot as pyplot
import pandas
import sys
import time

if __name__ == "__main__":

    number = sys.argv[1]

    try:

        data = pandas.io.parsers.read_csv( 
            "/Volumes/NO NAME/LOG{number}.TXT".format( number=number ),
            index_col=False,
            names=['time', 'raw_x', 'raw_y', 'raw_z', 
                   'dmp_x', 'dmp_y', 'dmp_z' ],
            na_values=['nan'],
        )
    except IOError:
        sys.exit("file was not found")
    
    
    
    p = data.plot( x='time' )


    font = {
        'family': 'Consolas',
        'weight': 'x-small',
        'size': 11.0,
        'stretch': 0
    }

    
    
    pyplot.rc( 'font', **font )
    pyplot.show( block=True )

    print data.head()
    
    
