//============================================================
// M10K module
//============================================================


module M10K_76800_8( //8 bit colors with 240 rows * 320 pixels/row
    output reg [7:0] q,
    input [7:0] d,
    input [16:0] write_address, read_address,
    input we, clk
);
	 // force M10K ram style
    reg [7:0] mem [76800:0]  /* synthesis ramstyle = "no_rw_check, M10K" */;

    always @ (posedge clk) begin
        if (we) begin
            mem[write_address] <= d;
        end
        q <= mem[read_address]; // q doesn't get d in this clock cycle
    end
endmodule
