uint8_t LTC681x_rdcv(uint8_t reg, // Controls which cell voltage register is read back.
                     uint8_t total_ic, // The number of ICs in the system
                     cell_asic *ic // Array of the parsed cell codes
                    )
{
	int8_t pec_error = 0;
	uint8_t *cell_data;
	uint8_t c_ic = 0;
	cell_data = (uint8_t *) malloc((NUM_RX_BYT*total_ic)*sizeof(uint8_t));

	if (reg == 0)
	{
		for (uint8_t cell_reg = 1; cell_reg<ic[0].ic_reg.num_cv_reg+1; cell_reg++) //Executes once for each of the LTC681x cell voltage registers
		{
			LTC681x_rdcv_reg(cell_reg, total_ic,cell_data );
			for (int current_ic = 0; current_ic<total_ic; current_ic++)
			{
			if (ic->isospi_reverse == false)
			{
			  c_ic = current_ic;
			}
			else
			{
			  c_ic = total_ic - current_ic - 1;
			}
			pec_error = pec_error + parse_cells(current_ic,cell_reg, cell_data,
												&ic[c_ic].cells.c_codes[0],
												&ic[c_ic].cells.pec_match[0]);
			}
		}
	}

	else
	{
		LTC681x_rdcv_reg(reg, total_ic,cell_data);

		for (int current_ic = 0; current_ic<total_ic; current_ic++)
		{
			if (ic->isospi_reverse == false)
			{
			c_ic = current_ic;
			}
			else
			{
			c_ic = total_ic - current_ic - 1;
			}
			pec_error = pec_error + parse_cells(current_ic,reg, &cell_data[8*c_ic],
											  &ic[c_ic].cells.c_codes[0],
											  &ic[c_ic].cells.pec_match[0]);
		}
	}
	LTC681x_check_pec(total_ic,CELL,ic);
	free(cell_data);

	return(pec_error);
}
