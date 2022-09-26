from __future__ import annotations

from numpy.random import normal
from random import randint

class Gene:
    def __init__(self, gene) -> None:
        self.gene = gene

class Chromosome:
    def __init__(self, genes: list[Gene]) -> None:
        self.genes = genes

        self.fitness: float = 0

class GeneticAlgorithm:
    def __init__(
        self, 
        population_size: int, 
        gene_randomizers: list[tuple[callable, tuple]], 
        mutation_std: float=0.1,
        reverse_population_sort=False
    ) -> None:
        """
        population_size: int
        gene_randomizers: list[tuple[callable, tuple]] -- [(randomizer, (min, max))]
        mutation_std: float
        reverse_population_sort: bool -- if True, higher fitness is better
        """

        self.population_size = population_size
        self.mutation_std = mutation_std
        self.gene_randomizers: list[tuple[callable, tuple]] = gene_randomizers  

        self.population: list[Chromosome] = []
        self.generation = 0
        self.reverse_population_sort = reverse_population_sort

        self.initalize_population()

    def initalize_population(self, count: int=None) -> list[Chromosome]:
        count = self.population_size if not count else count

        for _ in range(count):
            genes: list[Gene] = []
            for randomizer, r_range in self.gene_randomizers:
                genes.append(Gene(
                    gene=randomizer() * (r_range[1] - r_range[0]) + r_range[0]
                ))
            self.population.append(Chromosome(genes))

    def top_percent(self, percent: float) -> list[Chromosome]:
        self.population.sort(key=lambda x: x.fitness, reverse=self.reverse_population_sort)
        top_percent: list[Chromosome] = self.population[:int(self.population_size * percent)]

        # remove chromosomes that showed no indication of improvement
        i: int = int(self.population_size * percent)
        while i > 0:
            if not top_percent[i].fitness in [float('inf'), float('-inf')]:
                top_percent.pop(i)
            i -= 1

        return top_percent

    def crossover(self, parent1: Chromosome, parent2: Chromosome, split: int) -> tuple(Chromosome):
        child_1 = parent1.genes[:split] + parent2.genes[split:]
        child_2 = parent2.genes[:split] + parent1.genes[split:]
        return child_1, child_2

    def mutate_population(self, percent: float):
        """
        percent: float -- percent as a decimal of population to mutate
        """
        # set next gen populuation as top percent of current populuation
        self.population: list[Chromosome] = self.top_percent(percent)

        # add children of top percent to next gen population
        for i in range(0, len(self.population), 2):
            parent_1 = self.population[i]
            parent_2 = self.population[i + 1]
            child_1, child_2 = self.crossover(
                parent_1, parent_2, randint(1, len(parent_1.genes)-2)
            )

            # mutate children using normal distribution on each gene
            for child_1_gene, child_2_gene in zip(child_1, child_2):
                child_1_gene.gene += normal(scale=self.mutation_std)
                child_2_gene.gene += normal(scale=self.mutation_std)

            # add children to next gen population
            self.population.append(Chromosome(child_1))
            self.population.append(Chromosome(child_2))

        # add new random chromosomes to next gen population
        self.initalize_population(self.population_size - len(self.population))