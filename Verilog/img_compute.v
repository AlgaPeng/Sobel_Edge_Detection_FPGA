
module img_compute(
	input clk,
	input reset,
	input [7:0] rd_data,
	input val,
	output wr_en,
	output [7:0] wr_data,
	output reg [16:0] wr_addr
);

	reg [7:0] line_buffer [959:0]; // 320 * 3
	reg [1:0] counter;
	reg [1:0] edge_counter;
	reg [7:0] row_idx;
	reg [9:0] pixel_idx;
	reg [7:0] p0, p1, p2, p3, p5, p6, p7, p8;
   wire [7:0] out;
	
	//----------------------------------------------------------------------
	// State Definitions
	//----------------------------------------------------------------------

	localparam STATE_INIT = 4'd0;
	localparam STATE_WR   = 4'd1;
	localparam STATE_CAL0 = 4'd2;
	localparam STATE_CAL1 = 4'd3;
	localparam STATE_CAL2 = 4'd4;
	localparam STATE_UPDT = 4'd5;
	localparam STATE_PRE0 = 4'd6;
	localparam STATE_PRE1 = 4'd7;
	localparam STATE_PRE2 = 4'd8;

	//----------------------------------------------------------------------
	// State
	//----------------------------------------------------------------------

	reg [3:0] state_reg;
	reg [3:0] state_next;
	
	always @( posedge clk ) begin
		if ( reset ) begin
			state_reg <= STATE_INIT;
		end
		else begin
			if(val) state_reg <= state_next;
		end
	end

	//----------------------------------------------------------------------
	// State Transitions
	//----------------------------------------------------------------------
	always @( state_reg, pixel_idx, edge_counter, counter, row_idx ) begin

		 state_next = state_reg;
		 case ( state_reg )
		 
			STATE_INIT: begin
				state_next = STATE_WR;
			end
			
			STATE_WR: begin
				if (pixel_idx < 10'd640) state_next = STATE_WR; // if not reached 2 rows + 3 pixels, keep reading
				else state_next = STATE_PRE0;
			end
			
			STATE_PRE0: begin
				state_next = STATE_CAL0;
			end

			STATE_CAL0: begin
				if (pixel_idx < 10'd959) state_next = STATE_CAL0; // keep calculating
				else state_next = STATE_UPDT;
			end

			STATE_PRE1: begin
				state_next = STATE_CAL1;
			end

			STATE_CAL1: begin
				if (pixel_idx < 10'd319) state_next = STATE_CAL1; // keep calculating
				else state_next = STATE_UPDT;
			end

			STATE_PRE2: begin
				state_next = STATE_CAL2;
			end

			STATE_CAL2: begin
				if (pixel_idx < 10'd639) state_next = STATE_CAL2; // keep calculating
				else state_next = STATE_UPDT;
			end

			STATE_UPDT: begin
				if(counter == 2'd1) state_next = STATE_PRE1;
				else if(counter == 2'd2) state_next = STATE_PRE2;
				else if(counter == 2'd0) state_next = STATE_PRE0;
				// Go back to the beginning
				if (row_idx == 8'd237) state_next = STATE_INIT;
			end

			default: begin
				state_next = STATE_INIT;
			end
		endcase
	end

	//----------------------------------------------------------------------
	// Output Transitions
	//----------------------------------------------------------------------
	always @( posedge clk ) begin

		case ( state_reg )
			STATE_INIT: begin
				row_idx                <= 0;
				pixel_idx              <= 1;
            	counter                <= 1;
				line_buffer[0]         <= rd_data;
				wr_addr                <= -1;
			end

			STATE_WR: begin
				if(val) begin
					pixel_idx              <= pixel_idx + 1;
					line_buffer[pixel_idx] <= rd_data;
				end
			end

			//  line0: p0 p1 p2 ...
			//  line1: p3 p4 p5 ...
			//  line2: p6 p7 p8 ... (new data)
			STATE_PRE0: begin
				if(val) begin
					pixel_idx              <= pixel_idx + 1;
					line_buffer[pixel_idx] <= rd_data;
				end
			end
			
			//  line0: p0 p1 p2 ...
			//  line1: p3 p4 p5 ...
			//  line2: p6 p7 p8 ... (new data)
			STATE_CAL0: begin
				if(val) begin
					pixel_idx              <= pixel_idx + 1;
					line_buffer[pixel_idx] <= rd_data;
					wr_addr                <= wr_addr + 1;
					p0                     <= line_buffer[pixel_idx - 10'd642]; 
					p1                     <= line_buffer[pixel_idx - 10'd641]; 
					p2                     <= line_buffer[pixel_idx - 10'd640]; 
					p3                     <= line_buffer[pixel_idx - 10'd322]; 
					p5                     <= line_buffer[pixel_idx - 10'd320]; 
					p6                     <= line_buffer[pixel_idx - 10'd2];
					p7                     <= line_buffer[pixel_idx - 10'd1];
					p8                     <= rd_data;
				end
			end

			//  line3: p6 p7 p8 ... (new data)
			//  line1: p0 p1 p2 ... 
			//  line2: p3 p4 p5 ... 
			STATE_PRE1: begin
				if(val) begin
					pixel_idx              <= pixel_idx + 1;
					line_buffer[pixel_idx] <= rd_data;
				end
			end

			//  line3: p6 p7 p8 ... (new data)
			//  line1: p0 p1 p2 ... 
			//  line2: p3 p4 p5 ... 
			STATE_CAL1: begin
				if(val) begin
					pixel_idx              <= pixel_idx + 1;
					line_buffer[pixel_idx] <= rd_data;
					wr_addr                <= wr_addr + 1;
					p0                     <= line_buffer[pixel_idx + 10'd318]; 
					p1                     <= line_buffer[pixel_idx + 10'd319]; 
					p2                     <= line_buffer[pixel_idx + 10'd320]; 
					p3                     <= line_buffer[pixel_idx + 10'd638]; 
					p5                     <= line_buffer[pixel_idx + 10'd640]; 
					p6                     <= line_buffer[pixel_idx - 10'd2];
					p7                     <= line_buffer[pixel_idx - 10'd1];
					p8                     <= rd_data;
				end
			end
			
			//  line3: p3 p4 p5 ... 
			//  line4: p6 p7 p8 ... (new data)
			//  line2: p0 p1 p2 ... 
			STATE_PRE2: begin
				if(val) begin
					pixel_idx              <= pixel_idx + 1;
					line_buffer[pixel_idx] <= rd_data;
				end
			end

			//  line3: p3 p4 p5 ... 
			//  line4: p6 p7 p8 ... (new data)
			//  line2: p0 p1 p2 ... 
			STATE_CAL2: begin
				if(val) begin
					pixel_idx              <= pixel_idx + 1;
					line_buffer[pixel_idx] <= rd_data;
					wr_addr                <= wr_addr + 1;
					p0                     <= line_buffer[pixel_idx + 10'd318]; 
					p1                     <= line_buffer[pixel_idx + 10'd319]; 
					p2                     <= line_buffer[pixel_idx + 10'd320]; 
					p3                     <= line_buffer[pixel_idx - 10'd322]; 
					p5                     <= line_buffer[pixel_idx - 10'd320]; 
					p6                     <= line_buffer[pixel_idx - 10'd2];
					p7                     <= line_buffer[pixel_idx - 10'd1];
					p8                     <= rd_data;
				end
			end

			STATE_UPDT: begin
				if(val) begin
					// update the row index
					row_idx                <= row_idx + 1;

					// reset pixel index to be at the beginning of bottom row
					if(counter == 2'd0) begin 
						pixel_idx        <= 10'd641;
						line_buffer[640] <= rd_data;
					end
					// reset pixel index to be at the beginning of top row
					else if(counter == 2'd1) begin
						pixel_idx        <= 10'd1;
						line_buffer[0]   <= rd_data;
					end
					// reset pixel index to be at the beginning of top row
					else if(counter == 2'd2) begin
						pixel_idx        <= 10'd321;
						line_buffer[320] <= rd_data;
					end

					if(counter < 2'd2) begin
						counter            <= counter + 1;
					end
					else counter           <= 2'd0;
				end
			end
			
		endcase
	end
	
	// only enbale write signal when at calculation stages
	assign wr_en = (state_reg == STATE_CAL0 || state_reg == STATE_CAL1 || state_reg == STATE_CAL2 || row_idx == 8'd237) ? 1 : 0;
	assign wr_data = wr_en ? out : 8'b111_000_00;
	
	conv sobel_convolution( p0, p1, p2, p3, p5, p6, p7, p8, out);
	
endmodule