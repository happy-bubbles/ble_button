openocd -d2 -f x.cfg  -c "init; halt; nrf51 mass_erase; reset; halt; flash write_image erase $1 0; verify_image $1 0; reset; exit;"
