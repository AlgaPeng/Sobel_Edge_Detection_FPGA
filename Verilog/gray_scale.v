module pixel_converter (
  input [7:0] pixel_in,  // 8-bit color pixel input
  output [7:0] pixel_out  // 8-bit grayscale pixel output
);

	// Calculate grayscale value using formula：
	// Gray = 0.2989 * Red + 0.5870 * Green + 0.1140 * Blue
	assign pixel_out[1:0] = ((pixel_in[7:5] + pixel_in[4:2] + {pixel_in[1:0], 1'b0})/3) >> 1; //only take the most sigficant 2 bits for BLUE
	assign pixel_out[4:2] = (pixel_in[7:5] + pixel_in[4:2] + {pixel_in[1:0], 1'b0})/3;
	assign pixel_out[7:5] = (pixel_in[7:5] + pixel_in[4:2] + {pixel_in[1:0], 1'b0})/3;
//	assign pixel_out = 8'b111_000_00; //only take the most sigficant 2 bits for BLUE

endmodule