// parameters
#define SIGMA 0.5
#define ALPHA 8
#define ESS 13
#define KDEPTH 6
#define KNORMAL 11.6
#define MIN_SIZE 30

class Segmenter
{
	public:
		std::vector<candidate> segment(image<float>* img);

	private:
		image<float>* subsample(image<float>* img);
}
