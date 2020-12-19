## Path Generation Ruby file by Andrew Dobson

## Global Constants/Variables
LAT_TO_M = 0.00000904369
FLIGHT_Z = 3.2
WAIT_TIME = 3.0
$depth = 0
$rows = 0

$mp_wp_num = 1
$mp_fout
$csv_fout

## Generate a forward row
def forward_generate( row )
    real_x = row
    mp_x = LAT_TO_M * row
    for i in 0 .. $depth
        real_y = i
        mp_y = LAT_TO_M * i
        ## Output to our CSV
        $csv_fout << row.to_s << ", " << real_x << ", " << real_y << ", " << FLIGHT_Z << ", 1.5707963\n"
        ## Also output to MP
        $mp_fout << $mp_wp_num << " 0 0 16 "
        $mp_wp_num += 1
        ## Need to know if this is a first or last point of the row
        delay = 1.0
        if i == 0 or i == $depth
            delay = WAIT_TIME
        end
        $mp_fout << delay << " 0.1 0 0 " << ("%.12f" % mp_x) << " " << ("%.12f" % mp_y) << " " << FLIGHT_Z << " 1\n"
    end
end


## Generate a forward row
def backward_generate( row )
    real_x = row
    mp_x = LAT_TO_M * row
    for i in $depth.downto(0)
        real_y = i
        mp_y = LAT_TO_M * i
        ## Output to our CSV
        $csv_fout << row.to_s << ", " << real_x << ", " << real_y << ", " << FLIGHT_Z << ", 4.7123889\n"
        ## Also output to MP
        $mp_fout << $mp_wp_num << " 0 0 16 "
        $mp_wp_num += 1
        ## Need to know if this is a first or last point of the row
        delay = 1.0
        if i == 0 or i == $depth
            delay = WAIT_TIME
        end
        $mp_fout << delay << " 0.1 0 0 " << ("%.12f" % mp_x) << " " << ("%.12f" % mp_y) << " " << FLIGHT_Z << " 1\n"
    end
end

## Main Method
def main()
    ## Need some arguments for how large the generated path is going to be
    $depth = ARGV[0].to_i
    $rows = ARGV[1].to_i

    ## First, I need to make some preambles and open files
    $mp_fout = File.open("MP_waypoints_" + $depth.to_s + "_" + $rows.to_s + ".txt", "w") 
    $csv_fout = File.open("Pattern_" + $depth.to_s + "_" + $rows.to_s + ".csv", "w") 

    $mp_fout << "QGC WPL 110\n"
    $mp_fout << "0 1 0 16 1.0 0.1 0 0 0.0 0.0 " << FLIGHT_Z << " 1\n" 

    ## Then, for the number of rows
    for i in 1 .. $rows
        ## If this is an odd row, make a forward row
        if i%2 == 1
            forward_generate( i-1 )
        ## else if it is even, make a backward row
        else
            backward_generate( i-1 )
        end
    end
end


main()
